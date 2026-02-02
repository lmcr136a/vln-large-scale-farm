"""
Server to Panel Communication
Sending data from server to web panel (map, system info)
"""
import os
import time
import psutil
import shutil
import yaml
import cv2


class ServerToPanel:
    def __init__(self, socketio, config):
        self.socketio = socketio
        self.config = config
        self.map_update_counter = 0
        
    def get_wifi_name(self):
        """Get WiFi name"""
        try:
            result = os.popen("iwgetid -r").read().strip()
            return result if result else "Not connected"
        except Exception:
            return "Unavailable"
    
    def send_system_monitor(self, linear_speed):
        """Send system monitoring info"""
        data_dir = os.path.expanduser(self.config['paths']['data_dir'])
        os.makedirs(data_dir, exist_ok=True)
        
        total, used, free = shutil.disk_usage(data_dir)
        cpu = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory().percent
        wifi = self.get_wifi_name()
        
        self.socketio.emit('sysmon', {
            'cpu': cpu,
            'mem': mem,
            'used_gb': round(used / (1024**3), 1),
            'total_gb': round(total / (1024**3), 1),
            'used_pct': round((used / total) * 100, 1),
            'linear_mps': round(linear_speed, 2),
            'linear_mph': round(linear_speed * 2.23694, 2),
            'wifi': wifi,
        })
    
    def send_map_update(self):
        """Send map update info"""
        map_dir = os.path.expanduser(self.config['paths']['map_dir'])
        map_png = os.path.join(map_dir, 'map_latest.png')
        map_yaml = os.path.join(map_dir, 'map_latest.yaml')
        
        if not os.path.exists(map_png):
            return
        
        map_info = {
            'resolution': self.config['map']['default_resolution'],
            'origin_x': 0.0,
            'origin_y': 0.0,
            'robot_x': 0.0,
            'robot_y': 0.0,
            'robot_yaw': 0.0
        }
        
        if os.path.exists(map_yaml):
            with open(map_yaml, 'r') as f:
                yaml_data = yaml.safe_load(f)
                map_info['resolution'] = yaml_data.get('resolution', map_info['resolution'])
                origin = yaml_data.get('origin', [0.0, 0.0, 0.0])
                map_info['origin_x'] = origin[0]
                map_info['origin_y'] = origin[1]
                map_info['robot_x'] = yaml_data.get('robot_x', 0.0)
                map_info['robot_y'] = yaml_data.get('robot_y', 0.0)
                map_info['robot_yaw'] = yaml_data.get('robot_yaw', 0.0)
        
        img = cv2.imread(map_png)
        if img is None:
            return
        height, width = img.shape[:2]
        
        self.socketio.emit('map_update', {
            'info': {
                'resolution': map_info['resolution'],
                'origin_x': map_info['origin_x'],
                'origin_y': map_info['origin_y'],
                'width': width,
                'height': height,
                'robot_x': map_info['robot_x'],
                'robot_y': map_info['robot_y'],
                'robot_yaw': map_info['robot_yaw']
            }
        })
    
    def monitor_loop(self, linear_speed_getter):
        """Monitoring loop (runs in separate thread)"""
        while True:
            # Send system info
            self.send_system_monitor(linear_speed_getter())
            
            # Send map update (every 5 seconds)
            self.map_update_counter += 1
            if self.map_update_counter >= self.config['map']['update_interval']:
                self.send_map_update()
                self.map_update_counter = 0
            
            time.sleep(self.config['monitor']['update_interval'])