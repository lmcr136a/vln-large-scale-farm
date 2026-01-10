import os
import sys
import time
import threading
import numpy as np
from flask import Flask, send_file
from flask_socketio import SocketIO

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import psutil
import shutil
import yaml
import base64
import cv2

from subprocess import Popen, PIPE
import signal, atexit
from datetime import datetime

import autonomous_driving


MAX_REPEAT_NUM = 3

sys.stdout.reconfigure(encoding='utf-8')
sys.stderr.reconfigure(encoding='utf-8')

rclpy.init()
latest_rgb_image = None
node = rclpy.create_node('web_control_node')
pub = node.create_publisher(Twist, '/cmd_vel', 10)

def rgb_callback(msg):
    global latest_rgb_image
    try:
        print("🎥 RGB callback triggered!")
        height = msg.height
        width = msg.width
        channels = 3
        
        img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, channels)
        
        if msg.encoding == 'rgb8':
            img_array = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
        
        _, buffer = cv2.imencode('.jpg', img_array, [cv2.IMWRITE_JPEG_QUALITY, 80])
        latest_rgb_image = base64.b64encode(buffer).decode('utf-8')
        
        print(f"✅ Image encoded: {len(latest_rgb_image)} bytes")
    except Exception as e:
        print(f"❌ RGB callback error: {e}")
        import traceback
        traceback.print_exc()

rgb_sub = node.create_subscription(Image, '/rgb', rgb_callback, 10)
print("\n\n\n\n\n\n RRRRRRRRRR")


app = Flask(__name__)
socketio = SocketIO(
    app,
    cors_allowed_origins='*',
    ping_interval=5,
    ping_timeout=1,
    max_http_buffer_size=10000000
)

record_process = None
down_keys = set()
should_update_twist = True

pan = 0
tilt = 0
zoom = 0
ptz_lock = threading.Lock()
prev_pan = pan
prev_tilt = tilt
prev_zoom = zoom

pan_min, pan_max = -468000, 468000
tilt_min, tilt_max = -324000, 324000
zoom_min, zoom_max = 0, 100
pan_step = 3600 * 5
tilt_step = 7200 * 2
zoom_step = 10

linear_speed = 1.0
angular_speed = 0.5
MAX_LINEAR_SPEED = 1.5
MIN_LINEAR_SPEED = 0.1
MAX_ANGULAR_SPEED = 1.0
MIN_ANGULAR_SPEED = 0.1

current_frame_data = None
pending_recording_info = None
recording_active = False
recording_rgbd_path = None
lidar_paused = False   
last_stop_time = None
last_frame_save_time = 0

# Autonomous driving state
auto_mode_active = False
auto_thread = None
auto_stop_event = threading.Event()

def update_ptz():
    global pan, tilt, zoom, prev_pan, prev_tilt, prev_zoom
    updated = False
    with ptz_lock:
        if 'w' in down_keys:
            tilt = min(tilt + tilt_step, tilt_max)
            updated = True
        if 's' in down_keys:
            tilt = max(tilt - tilt_step, tilt_min)
            updated = True
        if 'a' in down_keys:
            pan = min(pan + pan_step, pan_max)
            updated = True
        if 'd' in down_keys:
            pan = max(pan - pan_step, pan_min)
            updated = True
        if 'x' in down_keys:
            zoom = min(zoom + zoom_step, zoom_max)
            updated = True
        if 'z' in down_keys:
            zoom = max(zoom - zoom_step, zoom_min)
            updated = True
        if updated and (pan != prev_pan or tilt != prev_tilt or zoom != prev_zoom):
            os.system(f"v4l2-ctl --set-ctrl=pan_absolute={pan} --set-ctrl=tilt_absolute={tilt} --set-ctrl=zoom_absolute={zoom}")
            prev_pan, prev_tilt, prev_zoom = pan, tilt, zoom

@app.route('/map_latest')
def serve_map():
    map_path = os.path.expanduser('~/box/vln-large-scale-farm/cartographer_ws/output/map_latest.png')
    if os.path.exists(map_path):
        return send_file(map_path, mimetype='image/png')
    else:
        return "Map not available", 404

@socketio.on('map_clicked')
def handle_map_clicked(data):
    img_x = data.get('img_x')
    img_y = data.get('img_y')
    world_x = data.get('world_x')
    world_y = data.get('world_y')
    print(f"🎯 Map clicked - Image: ({img_x}, {img_y}), World: ({world_x:.2f}, {world_y:.2f})")

@socketio.on('keydown')
def handle_keydown(data):
    global linear_speed, angular_speed, should_update_twist
    
    # Don't process manual controls in auto mode
    if auto_mode_active:
        return
    
    print(f"⬇️ Key down: {data}")
    down_keys.add(data)
    should_update_twist = True
    
    updated = False
    if data == ',':
        linear_speed = max(MIN_LINEAR_SPEED, linear_speed - 0.1)
        updated = True
    elif data == '.':
        linear_speed = min(MAX_LINEAR_SPEED, linear_speed + 0.1)
        updated = True
    elif data == '[':
        angular_speed = max(MIN_ANGULAR_SPEED, angular_speed - 0.1)
        updated = True
    elif data == ']':
        angular_speed = min(MAX_ANGULAR_SPEED, angular_speed + 0.1)
        updated = True

    if updated:
        socketio.emit('speed_update', {'linear': linear_speed, 'angular': angular_speed})
        print(f"Updated speeds: Linear={linear_speed}, Angular={angular_speed}")

@socketio.on('keyup')
def handle_keyup(data):
    # Don't process manual controls in auto mode
    if auto_mode_active:
        return
    
    print(f"⬆️ Key up: {data}")
    down_keys.discard(data)
    twist = Twist()
    pub.publish(twist)
    
@socketio.on('connect')
def handle_connect():
    global should_update_twist
    should_update_twist = True
    send_map_update()

@socketio.on('disconnect')
def handle_disconnect():
    clear_keys_and_stop()

def clear_keys_and_stop():
    global down_keys, should_update_twist
    down_keys.clear()
    should_update_twist = False
    twist = Twist()
    pub.publish(twist)


def autonomous_control_loop(robot_x, robot_y, robot_yaw, waypoints):
    global auto_mode_active
    
    socketio.emit('robot_status', {'status': f'Navigating to {len(waypoints)} waypoints'}, namespace='/')
    
    print(f"Starting autonomous driving from ({robot_x:.2f}, {robot_y:.2f})")
    print(f"Waypoints: {waypoints}")
    
    def get_robot_pose():
        map_yaml = os.path.expanduser('~/box/vln-large-scale-farm/cartographer_ws/output/map_latest.yaml')
        if os.path.exists(map_yaml):
            with open(map_yaml, 'r') as f:
                yaml_data = yaml.safe_load(f)
                x = yaml_data.get('robot_x', robot_x)
                y = yaml_data.get('robot_y', robot_y)
                yaw = yaml_data.get('robot_yaw', robot_yaw)
                return x, y, yaw
        return robot_x, robot_y, robot_yaw
    
    REPEAT_COUNT = 0
    try:
        while REPEAT_COUNT < MAX_REPEAT_NUM:
            for control_cmd in autonomous_driving.run(waypoints, get_robot_pose):
                if auto_stop_event.is_set():
                    print("Autonomous driving stopped by user")
                    socketio.emit('robot_status', {'status': 'Stopped by user'}, namespace='/')
                    break
                
                vt = control_cmd.get('vt', 0.0)
                vr = control_cmd.get('vr', 0.0)
                tt = control_cmd.get('tt', 0.0)
                tr = control_cmd.get('tr', 0.0)
                completed = control_cmd.get('completed', False)
                
                
                waypoint_idx = control_cmd.get('waypoint_reached')
                if waypoint_idx is not None:
                    print(f"🎯 Waypoint {waypoint_idx} reached, emitting event")
                    for _ in range(10):
                        socketio.emit('robot_status', {'status': f'Reached waypoint {waypoint_idx}'}, namespace='/')
                        socketio.sleep(0.1)
                        socketio.emit('waypoint_reached', {'index': waypoint_idx}, namespace='/')
                        socketio.sleep(0.1)
                        
                status = control_cmd.get('status')
                if status is not None:
                    print(f"📢 Emitting status: {status}")
                    for _ in range(5):
                        socketio.emit('robot_status', {'status': status}, namespace='/')
                        socketio.sleep(0.02)
                
                if abs(tt) > 0.001:
                    twist = Twist()
                    twist.linear.x = vt
                    for i in range(int(tt)):
                        pub.publish(twist)
                        socketio.sleep(0.5)
                    
                    twist = Twist()
                    pub.publish(twist)
                    socketio.sleep(0.1)
                
                if abs(tr) > 0.001:
                    twist = Twist()
                    twist.angular.z = vr
                    for i in range(int(tr)):
                        print(i)
                        pub.publish(twist)
                        socketio.sleep(0.2)
                    
                    twist = Twist()
                    pub.publish(twist)
                    socketio.sleep(0.1)
                
                if completed:
                    print("Path completed!")
                    for _ in range(5):
                        socketio.emit('robot_status', {'status': 'Path completed!'}, namespace='/')
                        socketio.sleep(0.1)
                        socketio.emit('auto_mode_completed', namespace='/')
                        socketio.sleep(0.1)
                    break
            REPEAT_COUNT += 1
                
    except Exception as e:
        print(f"Autonomous driving error: {e}")
        socketio.emit('robot_status', {'status': f'Error: {str(e)}'}, namespace='/')
    finally:
        twist = Twist()
        pub.publish(twist)
        auto_mode_active = False
        socketio.emit('robot_status', {'status': ''}, namespace='/')
        print("Autonomous driving finished")


@socketio.on('start_autonomous')
def handle_start_autonomous(data):
    global auto_mode_active, auto_thread
    
    if auto_mode_active:
        print("⚠️ Autonomous mode already active")
        socketio.emit('robot_status', {'status': 'Already running'})
        return
    
    robot_x = data.get('robot_x', 0.0)
    robot_y = data.get('robot_y', 0.0)
    robot_yaw = data.get('robot_yaw', 0.0)
    waypoints = data.get('waypoints', [])
    
    if len(waypoints) < 2:
        print("⚠️ Need at least 2 waypoints")
        socketio.emit('robot_status', {'status': 'Need 2+ waypoints'})
        return
    
    waypoint_list = [(wp['x'], wp['y']) for wp in waypoints]
    
    auto_mode_active = True
    auto_stop_event.clear()
    
    socketio.emit('robot_status', {'status': 'Starting autonomous mode...'})
    
    socketio.start_background_task(
        autonomous_control_loop,
        robot_x, robot_y, robot_yaw, waypoint_list
    )
    
    print(f"🚀 Autonomous mode started with {len(waypoint_list)} waypoints")
    
    
@socketio.on('stop_autonomous')
def handle_stop_autonomous():
    global auto_mode_active
    
    if not auto_mode_active:
        print("⚠️ Autonomous mode not active")
        return
    
    print("🛑 Stopping autonomous mode")
    auto_stop_event.set()
    auto_mode_active = False
    
    # Stop the robot immediately
    twist = Twist()
    pub.publish(twist)

def update_loop():
    global should_update_twist
    while True:
        # Skip manual control during autonomous mode
        if auto_mode_active:
            time.sleep(0.1)
            continue
            
        if not should_update_twist:
            time.sleep(0.1)
            continue
        update_ptz()
        twist = Twist()
        if 'ArrowUp' in down_keys:
            twist.linear.x += linear_speed
        if 'ArrowDown' in down_keys:
            twist.linear.x -= linear_speed
        if 'ArrowLeft' in down_keys:
            twist.angular.z += angular_speed
        if 'ArrowRight' in down_keys:
            twist.angular.z -= angular_speed
        pub.publish(twist)
        time.sleep(0.1)

def ros_spin():
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

def get_wifi_name():
    try:
        result = os.popen("iwgetid -r").read().strip()
        return result if result else "Not connected"
    except Exception:
        return "Unavailable"

def send_map_update():
    map_png = os.path.expanduser('~/box/vln-large-scale-farm/cartographer_ws/output/map_latest.png')  
    map_yaml = os.path.expanduser('~/box/vln-large-scale-farm/cartographer_ws/output/map_latest.yaml')
    
    if not os.path.exists(map_png):
        return
    
    map_info = {
        'resolution': 0.05,
        'origin_x': 0.0,
        'origin_y': 0.0,
        'robot_x': 0.0,
        'robot_y': 0.0,
        'robot_yaw': 0.0
    }
    
    if os.path.exists(map_yaml):
        with open(map_yaml, 'r') as f:
            yaml_data = yaml.safe_load(f)
            map_info['resolution'] = yaml_data.get('resolution', 0.05)
            origin = yaml_data.get('origin', [0.0, 0.0, 0.0])
            map_info['origin_x'] = origin[0]
            map_info['origin_y'] = origin[1]
            map_info['robot_x'] = yaml_data.get('robot_x', 0.0)
            map_info['robot_y'] = yaml_data.get('robot_y', 0.0)
            map_info['robot_yaw'] = yaml_data.get('robot_yaw', 0.0)
    
    img = cv2.imread(map_png)
    if img is None:
        print("⚠️ Failed to read map image")
        return
    height, width = img.shape[:2]
    
    socketio.emit('map_update', {
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
        
    
def system_monitor():
    map_update_counter = 0
    while True:
        data_dir = os.path.expanduser("~/box/vln-large-scale-farm/data")
        os.makedirs(data_dir, exist_ok=True)
        total, used, free = shutil.disk_usage(data_dir)
        cpu = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory().percent
        wifi = get_wifi_name()
        
        socketio.emit('sysmon', {
            'cpu': cpu,
            'mem': mem,
            'used_gb': round(used / (1024**3), 1),
            'total_gb': round(total / (1024**3), 1),
            'used_pct': round((used / total) * 100, 1),
            'linear_mps': round(linear_speed, 2),
            'linear_mph': round(linear_speed * 2.23694, 2),
            'wifi': wifi,
        })
        
        if latest_rgb_image:
            socketio.emit('rgb_frame', {'image': latest_rgb_image})

        map_update_counter += 1
        if map_update_counter >= 5:
            send_map_update()
            map_update_counter = 0
        
        time.sleep(1)

@socketio.on('save_frame')
def handle_save_frame(image_data):
    global current_frame_data, pending_recording_info, recording_active, recording_rgbd_path, last_frame_save_time
    current_frame_data = image_data
    
    if pending_recording_info:
        filename = pending_recording_info
        pending_recording_info = None
        do_start_recording(filename)
        
    elif recording_active and recording_rgbd_path:
        current_time = time.time()
        if current_time - last_frame_save_time < 2.0:
            return
        last_frame_save_time = current_time
        
        rgb_image_path = os.path.join(recording_rgbd_path, "rgb.png")
        frame_bytes = image_data if isinstance(image_data, bytes) else image_data.encode()
        with open(rgb_image_path, 'wb') as f:
            f.write(frame_bytes)
        print(f"📸 Frame saved: {rgb_image_path}")
        
        
def do_start_recording(filename):
    global record_process, current_frame_data, recording_active, recording_rgbd_path
    
    print(f"📥 Start recording: {filename}")
    base_path = os.path.expanduser(f"~/box/vln-large-scale-farm/data/{filename}")
    timestamp_dir = datetime.now().strftime("%Y%m%d_%H%M")
    full_path = os.path.join(base_path, timestamp_dir)
    rgbd_path = os.path.join(full_path, "RGB-D")
    lidar_path = os.path.join(full_path, "LiDAR")   

    for path in [base_path, full_path, rgbd_path, lidar_path]:
        os.makedirs(path, exist_ok=True)

    recording_active = True
    recording_rgbd_path = rgbd_path

    if current_frame_data:
        rgb_image_path = os.path.join(rgbd_path, "rgb.png")
        frame_bytes = current_frame_data if isinstance(current_frame_data, bytes) else current_frame_data.encode()
        with open(rgb_image_path, 'wb') as f:
            f.write(frame_bytes)
        print(f"📸 Frame saved: {rgb_image_path}")

    sync_time = time.time()
    with open(os.path.join(full_path, "sync_time.txt"), "w") as f:
        f.write(str(sync_time))
    print(f"[SYNC] sync_time.txt written: {sync_time}")

    zed_cmd = f"python3 ~/box/vln-large-scale-farm/tools_zed/data_svo_sync.py --output '{rgbd_path}' --sync_time {sync_time}"
    config_path = os.path.expanduser("~/box/vln-large-scale-farm/cartographer_ws/src/livox_ros_driver2/config/mid360_config.json")
    # livox_cmd = f"cartographer_ws/src/livox_ros_driver2/~!/Livox-SDK2/record_lidar_sync/build/recorder_sync '{config_path}' '{lidar_path}'"

    zed_process = Popen(["bash", "-c", zed_cmd], stdout=PIPE, stderr=PIPE)
    # livox_process = Popen(["bash", "-c", livox_cmd], stdout=PIPE, stderr=PIPE)

    # record_process = (zed_process, livox_process)


@socketio.on('start_recording')
def handle_start_recording(filename):
    global record_process
    if record_process is not None:
        print("⚠️ Recording already running")
        return

    print(f"📥 Start recording: {filename}")
    base_path = os.path.expanduser(f"~/box/vln-large-scale-farm/data/{filename}")
    timestamp_dir = datetime.now().strftime("%Y%m%d_%H%M")
    full_path = os.path.join(base_path, timestamp_dir)
    rgbd_path = os.path.join(full_path, "RGB-D")
    lidar_path = os.path.join(full_path, "LiDAR")   

    for path in [base_path, full_path, rgbd_path, lidar_path]:
        os.makedirs(path, exist_ok=True)

    sync_time = time.time()
    with open(os.path.join(full_path, "sync_time.txt"), "w") as f:
        f.write(str(sync_time))

    zed_cmd = f"python3 ~/box/vln-large-scale-farm/tools_zed/data_svo_sync.py --output '{rgbd_path}' --sync_time {sync_time}"
    config_path = os.path.expanduser("~/box/vln-large-scale-farm/cartographer_ws/src/livox_ros_driver2/config/mid360_config.json")
    # livox_cmd = f"cartographer_ws/src/livox_ros_driver2/~!/Livox-SDK2/record_lidar_sync/build/recorder_sync '{config_path}' '{lidar_path}'"

    zed_process = Popen(["bash", "-c", zed_cmd], stdout=PIPE, stderr=PIPE)
    # livox_process = Popen(["bash", "-c", livox_cmd], stdout=PIPE, stderr=PIPE)

    time.sleep(2)
    if zed_process.poll() is not None:
        stdout, stderr = zed_process.communicate()
        print(f"[ERROR] ZED process died!")
        print(f"[STDERR] {stderr.decode('utf-8', errors='replace')}")

    # record_process = (zed_process, livox_process)

@socketio.on('stop_recording')
def handle_stop_recording():
    global record_process
    if not record_process:
        print("⚠️ No recording in progress")
        return

    zed_process, livox_process = record_process
    print("🛑 Stopping ZED & Livox recording")
    zed_process.send_signal(signal.SIGINT)
    try:
        stdout, stderr = zed_process.communicate(timeout=10)
        print(f"[ZED] STDOUT:\n{stdout.decode()}\nSTDERR:\n{stderr.decode()}")
    except Exception as e:
        print(f"⚠️ ZED stop error: {e}")

    livox_process.terminate()
    livox_process.wait()
    record_process = None


atexit.register(clear_keys_and_stop)
signal.signal(signal.SIGHUP, lambda s, f: clear_keys_and_stop())

threading.Thread(target=system_monitor, daemon=True).start()
threading.Thread(target=ros_spin, daemon=True).start()
threading.Thread(target=update_loop, daemon=True).start()

if __name__ == '__main__':
    os.makedirs(os.path.expanduser("~/box/vln-large-scale-farm/data"), exist_ok=True)
    print("✅ WebSocket control server running on port 5000...")
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True, use_reloader=False)