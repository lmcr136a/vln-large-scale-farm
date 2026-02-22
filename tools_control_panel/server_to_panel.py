"""
Server to Panel Communication
Map is sent only when hash changes; robot pose is sent separately at 10Hz
"""
import os
import time
import base64
import hashlib
import threading
import psutil
import shutil
import yaml
import cv2


# Maximum map dimension (px) for downscaling before sending - optimized for Tailscale
MAP_MAX_DIM = 1500


class ServerToPanel:
    def __init__(self, socketio, config):
        self.socketio = socketio
        self.config   = config
        self._last_map_hash = None

    # ── Internal Utilities ───────────────────────────────────────────────

    def get_wifi_name(self):
        try:
            result = os.popen("iwgetid -r").read().strip()
            return result if result else "Not connected"
        except Exception:
            return "Unavailable"

    def _read_map_yaml(self):
        """Read map metadata and robot pose from map_latest.yaml"""
        map_dir  = os.path.expanduser(self.config['paths']['map_dir'])
        map_yaml = os.path.join(map_dir, 'map_latest.yaml')

        info = {
            'resolution': 0.05,
            'origin_x': 0.0, 'origin_y': 0.0,
            'robot_x':  0.0, 'robot_y':  0.0, 'robot_yaw': 0.0,
        }
        if os.path.exists(map_yaml):
            with open(map_yaml, 'r') as f:
                d = yaml.safe_load(f)
            info['resolution'] = d.get('resolution', info['resolution'])
            origin = d.get('origin', [0.0, 0.0, 0.0])
            info['origin_x']   = origin[0]
            info['origin_y']   = origin[1]
            info['robot_x']    = d.get('robot_x',   0.0)
            info['robot_y']    = d.get('robot_y',   0.0)
            info['robot_yaw']  = d.get('robot_yaw', 0.0)
        return info

    # ── System Monitor ────────────────────────────────────────────────────

    def send_system_monitor(self, linear_speed):
        data_dir = os.path.expanduser(self.config['paths']['data_dir'])
        os.makedirs(data_dir, exist_ok=True)

        total, used, _ = shutil.disk_usage(data_dir)
        cpu  = psutil.cpu_percent(interval=None)
        mem  = psutil.virtual_memory().percent
        wifi = self.get_wifi_name()

        self.socketio.emit('sysmon', {
            'cpu':        cpu,
            'mem':        mem,
            'used_gb':    round(used  / (1024**3), 1),
            'total_gb':   round(total / (1024**3), 1),
            'used_pct':   round((used / total) * 100, 1),
            'linear_mps': round(linear_speed, 2),
            'linear_mph': round(linear_speed * 2.23694, 2),
            'wifi':       wifi,
        })

    # ── Map Transmission (hash comparison - only send on change) ──────────

    def send_map_update(self, force=False):
        """
        Read map PNG, compare hash, and send as base64 only if changed.
        force=True bypasses hash check and always sends (used on initial connect).
        """
        map_dir = os.path.expanduser(self.config['paths']['map_dir'])
        map_png = os.path.join(map_dir, 'map_latest.png')

        if not os.path.exists(map_png):
            return

        # Read file
        with open(map_png, 'rb') as f:
            raw = f.read()

        new_hash = hashlib.md5(raw).hexdigest()
        if not force and new_hash == self._last_map_hash:
            return   # No change - skip transmission
        self._last_map_hash = new_hash

        # Decode with OpenCV and downscale (e.g. 6000×6000 → max 1500px)
        img = cv2.imdecode(
            __import__('numpy').frombuffer(raw, dtype='uint8'),
            cv2.IMREAD_COLOR
        )
        if img is None:
            return

        h, w = img.shape[:2]
        max_dim = MAP_MAX_DIM
        if max(h, w) > max_dim:
            scale = max_dim / max(h, w)
            img = cv2.resize(img,
                             (int(w * scale), int(h * scale)),
                             interpolation=cv2.INTER_AREA)
            h, w = img.shape[:2]

        # JPEG encode at quality 60 (maps need more clarity than RGB stream)
        _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 60])
        b64 = base64.b64encode(buf).decode('utf-8')

        # Read map metadata
        info = self._read_map_yaml()

        self.socketio.emit('map_image', {
            'image':      b64,
            'width':      w,
            'height':     h,
            'resolution': info['resolution'],
            'origin_x':   info['origin_x'],
            'origin_y':   info['origin_y'],
        })
        print(f"[map] sent {w}×{h} ({len(b64)//1024} KB)")

    # ── Robot Pose Transmission (10~20 Hz) ───────────────────────────────

    def send_robot_pose(self):
        """Read only robot position from yaml and emit lightweight pose event"""
        info = self._read_map_yaml()
        self.socketio.emit('robot_pose', {
            'x':   info['robot_x'],
            'y':   info['robot_y'],
            'yaw': info['robot_yaw'],
        })

    # ── Main Loop ─────────────────────────────────────────────────────────

    def monitor_loop(self, linear_speed_getter):
        """
        - sysmon:     every monitor.update_interval seconds (default 0.25s = 4Hz)
        - robot_pose: every 0.1s (10Hz)
        - map_image:  every map.update_interval counter tick, only if hash changed
        """
        pose_interval    = 0.1   # 10 Hz
        sysmon_interval  = self.config['monitor']['update_interval']
        map_check_count  = max(1, int(self.config['map']['update_interval'] / pose_interval))

        last_sysmon = 0.0
        counter     = 0

        while True:
            now = time.time()

            # Robot pose (every loop iteration = 10Hz)
            self.send_robot_pose()

            # System monitor
            if now - last_sysmon >= sysmon_interval:
                self.send_system_monitor(linear_speed_getter())
                last_sysmon = now

            # Map (check for changes at configured interval)
            counter += 1
            if counter >= map_check_count:
                self.send_map_update()
                counter = 0

            time.sleep(pose_interval)