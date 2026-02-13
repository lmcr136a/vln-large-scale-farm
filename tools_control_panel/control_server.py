#!/usr/bin/env python3
"""
Control Server - Main Entry Point
"""
import os
import sys
import yaml
import threading
import signal
import atexit

from flask import Flask, send_file, request
from flask_socketio import SocketIO

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Custom modules
from server_to_panel import ServerToPanel
from map_creation import MapCreationController
from get_path import PathPlanner
from autonomous_mode import AutonomousController
from recorder import MultiSensorRecorder


class ControlServer:
    def __init__(self, config_path='control_config.yaml'):
        self.shutdown_flag = threading.Event()
        
        # Load configuration
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        print("Configuration loaded")
        
        # Initialize ROS2
        rclpy.init()
        self.node = rclpy.create_node(self.config['ros2']['node_name'])
        self.cmd_vel_pub = self.node.create_publisher(
            Twist, 
            self.config['ros2']['cmd_vel_topic'], 
            10
        )
        
        print("ROS2 node initialized")
        
        # Initialize Flask & SocketIO
        self.app = Flask(__name__)
        self.socketio = SocketIO(
            self.app,
            cors_allowed_origins='*',
            ping_interval=self.config['server']['ping_interval'],
            ping_timeout=self.config['server']['ping_timeout'],
            max_http_buffer_size=self.config['server']['max_http_buffer_size']
        )
        print("Flask & SocketIO initialized")
        
        # Initialize MultiSensorRecorder
        print("Initializing MultiSensorRecorder...")
        base_data_dir = os.path.expanduser(self.config['paths']['data_dir'])
        self.recorder = MultiSensorRecorder(self.node, output_base_dir=base_data_dir)
        # Add ZED cameras to recorder
        sample_interval = self.config['recording'].get('sample_interval_sec', 300)  # Default 5 minutes
        
        self.zed_front = self.recorder.add_zed_camera(
            serial_number=48335070,
            name="front",
            socketio=self.socketio,
            sample_interval_sec=sample_interval,
            always_stream=True
        )
        
        self.zed_back = self.recorder.add_zed_camera(
            serial_number=49537850,
            name="back",
            socketio=self.socketio,
            sample_interval_sec=sample_interval,
            always_stream=True
        )
        
        print("ZED cameras initialized and streaming")
        
        # Initialize modules
        self.server_to_panel = ServerToPanel(self.socketio, self.config)
        self.map_controller = MapCreationController(self.cmd_vel_pub, self.config)
        self.path_planner = PathPlanner(self.config)
        self.auto_controller = AutonomousController(
            self.cmd_vel_pub, 
            self.socketio, 
            self.config
        )
        print("All modules initialized")
        
        # Recording state
        self.is_recording = False
        self.current_session_dir = None
        
        # Register routes and event handlers
        self.register_routes()
        self.register_socketio_events()
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Cleanup on exit
        atexit.register(self.cleanup)
    
    def signal_handler(self, sig, frame):
        """Handle shutdown signals"""
        print(f"\n\nReceived signal {sig}, shutting down gracefully...")
        self.shutdown_flag.set()
        self.cleanup()
        sys.exit(0)
        
    def register_routes(self):
        """Register Flask routes"""
        
        @self.app.route('/')
        @self.app.route('/control.html')
        def serve_control():
            return send_file('control.html')
        
        @self.app.route('/control.js')
        def serve_js():
            return send_file('control.js', mimetype='application/javascript')
        
        @self.app.route('/styles.css')
        def serve_css():
            return send_file('styles.css', mimetype='text/css')
        
        @self.app.route('/map_latest')
        def serve_map():
            map_dir = os.path.expanduser(self.config['paths']['map_dir'])
            map_path = os.path.join(map_dir, 'map_latest.png')
            if os.path.exists(map_path):
                return send_file(map_path, mimetype='image/png')
            else:
                return "Map not available", 404
                
        @self.app.route('/front_rgb')
        def serve_front_rgb():
            frame = self.zed_front.get_latest_frame()
            if frame:
                from io import BytesIO
                return send_file(BytesIO(frame), mimetype='image/jpeg')
            else:
                return "No frame", 404

        @self.app.route('/back_rgb')
        def serve_back_rgb():
            frame = self.zed_back.get_latest_frame()
            if frame:
                from io import BytesIO
                return send_file(BytesIO(frame), mimetype='image/jpeg')
            else:
                return "No frame", 404
            
    def register_socketio_events(self):
        """Register SocketIO event handlers"""
        
        @self.socketio.on('connect')
        def handle_connect():
            print(f"✅ Client connected: {request.sid}")
            self.map_controller.should_update_twist = True
            self.server_to_panel.send_map_update()
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            print(f"❌ Client disconnected: {request.sid}")
            try:
                self.map_controller.clear_keys_and_stop()
            except Exception as e:
                print(f"Error during disconnect cleanup: {e}")
        
        @self.socketio.on('heartbeat')
        def handle_heartbeat():
            pass
        
        @self.socketio.on('keydown')
        def handle_keydown(data):
            if not self.auto_controller.is_active():
                self.map_controller.handle_keydown(data, self.socketio)
        
        @self.socketio.on('keyup')
        def handle_keyup(data):
            if not self.auto_controller.is_active():
                self.map_controller.handle_keyup(data)
        
        @self.socketio.on('map_clicked')
        def handle_map_clicked(data):
            self.path_planner.handle_map_clicked(data)
        
        @self.socketio.on('start_autonomous')
        def handle_start_autonomous(data):
            robot_x = data.get('robot_x', 0.0)
            robot_y = data.get('robot_y', 0.0)
            robot_yaw = data.get('robot_yaw', 0.0)
            waypoints = data.get('waypoints', [])
            
            self.auto_controller.start(robot_x, robot_y, robot_yaw, waypoints)
        
        @self.socketio.on('stop_autonomous')
        def handle_stop_autonomous():
            self.auto_controller.stop()
        
        @self.socketio.on('start_recording')
        def handle_start_recording(dirname):
            """Start multi-sensor recording (ZED SVO2 + LiDAR)"""
            print(f"🔴 Received start_recording: {dirname}")
            
            if self.is_recording:
                print("⚠️ Already recording, stopping previous session first")
                self.recorder.stop_recording()
                self.is_recording = False
            
            if not dirname:
                print("❌ Invalid directory name for recording")
                return
            
            from datetime import datetime
            
            # Create session with custom name
            timestamp = datetime.now().strftime("%Y%m%d_%H%M")
            session_name = f"{dirname}/{timestamp}"
            
            try:
                self.current_session_dir = self.recorder.create_session(session_name)
                self.recorder.start_recording()
                self.is_recording = True
                
                print(f"✅ Recording started to: {self.current_session_dir}")
                print(f"   - Front ZED: rgbd_front.svo2")
                print(f"   - Back ZED: rgbd_back.svo2")
                print(f"   - LiDAR: lidar_pointcloud.bin")
                print(f"   - LiDAR IMU: lidar_imu.bin")
                print(f"   - Samples: Every {self.config['recording'].get('sample_interval_sec', 300)}s")
                
            except Exception as e:
                print(f"❌ Failed to start recording: {e}")
                self.is_recording = False
        
        @self.socketio.on('stop_recording')
        def handle_stop_recording():
            """Stop multi-sensor recording"""
            print("⏹️ Stopping recording...")
            
            if not self.is_recording:
                print("⚠️ No active recording session")
                return
            
            try:
                self.recorder.stop_recording()
                self.is_recording = False
                
                print("✅ Recording stopped")
                if self.current_session_dir:
                    print(f"📁 Data saved to: {self.current_session_dir}")
                    print("\n📊 Dataset structure:")
                    print("   20YYMMDD_HHMM/")
                    print("   ├── rgbd_front.svo2")
                    print("   ├── rgbd_back.svo2")
                    print("   ├── lidar_pointcloud.bin")
                    print("   ├── lidar_imu.bin")
                    print("   └── samples/")
                    print("       ├── MMDD_HHMM_front.jpg")
                    print("       ├── MMDD_HHMM_front_depth.png")
                    print("       ├── MMDD_HHMM_back.jpg")
                    print("       └── MMDD_HHMM_back_depth.png")
                    
                self.current_session_dir = None
                
            except Exception as e:
                print(f"❌ Error stopping recording: {e}")
    
    def ros_spin(self):
        """ROS2 spin loop"""
        while rclpy.ok() and not self.shutdown_flag.is_set():
            rclpy.spin_once(self.recorder.ros_node, timeout_sec=0.1)
    
    def cleanup(self):
        """Cleanup on exit"""
        if self.shutdown_flag.is_set():
            return
        
        print("\n🧹 Cleaning up...")
        self.shutdown_flag.set()
        
        # Stop robot movement
        try:
            self.map_controller.clear_keys_and_stop()
        except Exception as e:
            print(f"Error stopping robot: {e}")
        
        # Stop autonomous mode
        try:
            if self.auto_controller.is_active():
                self.auto_controller.stop()
        except Exception as e:
            print(f"Error stopping autonomous mode: {e}")
        
        # Stop recording if active
        try:
            if self.is_recording:
                print("Stopping active recording session...")
                self.recorder.stop_recording()
                self.is_recording = False
        except Exception as e:
            print(f"Error stopping recording: {e}")
        
        # Shutdown recorder
        try:
            print("Shutting down MultiSensorRecorder...")
            self.recorder.shutdown()
            print("Recorder shutdown complete")
        except Exception as e:
            print(f"Error during recorder shutdown: {e}")
        
        print("✅ Cleanup complete")
    
    def run(self):
        """Run server"""
        # Start background threads
        threading.Thread(
            target=self.server_to_panel.monitor_loop,
            args=(self.map_controller.get_linear_speed,),
            daemon=True
        ).start()
        
        threading.Thread(
            target=self.ros_spin,
            daemon=True
        ).start()
        
        threading.Thread(
            target=self.map_controller.update_loop,
            args=(self.auto_controller.is_active,),
            daemon=True
        ).start()
        
        # Create data directory
        data_dir = os.path.expanduser(self.config['paths']['data_dir'])
        os.makedirs(data_dir, exist_ok=True)
        
        # Run server
        print(f"🚀 Control server running on port {self.config['server']['port']}...")
        print(f"📊 Recording Format: SVO2 (H.265 compressed)")
        print(f"   - ZED Cameras: 30 FPS (compressed in SVO2)")
        print(f"   - LiDAR: ~{self.config['recording']['lidar_hz']}Hz")
        print(f"   - IMU: ~{self.config['recording']['imu_hz']}Hz")
        print(f"   - Samples: Every {self.config['recording'].get('sample_interval_sec', 300)}s")
        
        try:
            self.socketio.run(
                self.app,
                host=self.config['server']['host'],
                port=self.config['server']['port'],
                allow_unsafe_werkzeug=True,
                use_reloader=False
            )
        except KeyboardInterrupt:
            print("\n⚠️ Keyboard interrupt received")
        finally:
            self.cleanup()


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')
    
    server = ControlServer('control_config.yaml')
    server.run()