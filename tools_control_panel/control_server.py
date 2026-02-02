#!/usr/bin/env python3
"""
Control Server - Main Entry Point
Main server connecting web panel and robot
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
from zed_dual_camera import ZEDCameraRecorder


class ControlServer:
    def __init__(self, config_path='control_config.yaml'):
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
        
        # Initialize ZED cameras (after socketio is created)
        print("Initializing ZED cameras...")
        self.zed_front = ZEDCameraRecorder(
            serial_number=48335070,
            name="front",
            ros_node=self.node,
            socketio=self.socketio,
            interval=1.0,
            always_stream=True
        )
        self.zed_back = ZEDCameraRecorder(
            serial_number=49537850,
            name="back",
            ros_node=self.node,
            socketio=self.socketio,
            interval=1.0,
            always_stream=True
        )
        
        # Start ZED camera threads
        self.zed_front.start()
        self.zed_back.start()
        print("ZED cameras started")
        
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
        
        # Register routes and event handlers
        self.register_routes()
        self.register_socketio_events()
        
        # Cleanup on exit
        atexit.register(self.cleanup)
        signal.signal(signal.SIGHUP, lambda s, f: self.cleanup())
    
    def register_routes(self):
        """Register Flask routes"""
        @self.app.route('/map_latest')
        def serve_map():
            map_dir = os.path.expanduser(self.config['paths']['map_dir'])
            map_path = os.path.join(map_dir, 'map_latest.png')
            if os.path.exists(map_path):
                return send_file(map_path, mimetype='image/png')
            else:
                return "Map not available", 404
    
    def register_socketio_events(self):
        """Register SocketIO event handlers"""
        
        @self.socketio.on('connect')
        def handle_connect():
            print(f"Client connected: {request.sid}")
            self.map_controller.should_update_twist = True
            self.server_to_panel.send_map_update()
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            print(f"Client disconnected: {request.sid}")
            self.map_controller.clear_keys_and_stop()
        
        @self.socketio.on('heartbeat')
        def handle_heartbeat():
            pass  # Just acknowledge
        
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
            """Start ZED camera recording"""
            if not dirname:
                print("Invalid directory name for recording")
                return
            
            # Build full output path
            from datetime import datetime
            base_path = os.path.expanduser(self.config['paths']['data_dir'])
            timestamp_dir = datetime.now().strftime("%Y%m%d_%H%M")
            full_path = os.path.join(base_path, dirname, timestamp_dir)
            
            print(f"Starting ZED recording to: {full_path}")
            
            # Call ZED camera methods directly
            self.zed_front.start_recording(full_path)
            self.zed_back.start_recording(full_path)
            
            print("ZED recording started")
        
        @self.socketio.on('stop_recording')
        def handle_stop_recording():
            """Stop ZED camera recording"""
            print("Stopping ZED recording")
            
            # Call ZED camera methods directly
            self.zed_front.stop_recording()
            self.zed_back.stop_recording()
            
            print("ZED recording stopped")
    
    def ros_spin(self):
        """ROS2 spin loop"""
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
    
    def cleanup(self):
        """Cleanup on exit"""
        print("Cleaning up...")
        self.map_controller.clear_keys_and_stop()
        if self.auto_controller.is_active():
            self.auto_controller.stop()
        
        # Stop ZED cameras
        print("Stopping ZED cameras...")
        self.zed_front.stop()
        self.zed_back.stop()
    
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
        print(f"Control server running on port {self.config['server']['port']}...")
        self.socketio.run(
            self.app,
            host=self.config['server']['host'],
            port=self.config['server']['port'],
            allow_unsafe_werkzeug=True,
            use_reloader=False
        )


if __name__ == '__main__':
    # Set UTF-8 encoding
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')
    
    server = ControlServer('control_config.yaml')
    server.run()