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
from geometry_msgs.msg import Twist

from server_to_panel import ServerToPanel
from map_creation import MapCreationController
from get_path import PathPlanner
from autonomous_mode import AutonomousController
from recorder import MultiSensorRecorder


class ControlServer:
    def __init__(self, config_path='control_config.yaml'):
        self.shutdown_flag = threading.Event()

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        print("Configuration loaded")

        # ROS2
        rclpy.init()
        self.node = rclpy.create_node(self.config['ros2']['node_name'])
        self.cmd_vel_pub = self.node.create_publisher(
            Twist, self.config['ros2']['cmd_vel_topic'], 10
        )
        print("ROS2 node initialized")

        # ★ async_mode='threading' - HTTP requests and SocketIO events no longer block each other
        self.app = Flask(__name__)
        self.socketio = SocketIO(
            self.app,
            cors_allowed_origins='*',
            async_mode='threading',
            ping_interval=self.config['server']['ping_interval'],
            ping_timeout=self.config['server']['ping_timeout'],
            max_http_buffer_size=self.config['server']['max_http_buffer_size'],
            allow_upgrades=False,   # Disable WebSocket upgrade - Tailscale breaks WS headers
        )
        print("Flask & SocketIO initialized")

        print("Initializing MultiSensorRecorder...")
        base_data_dir = os.path.expanduser(self.config['paths']['data_dir'])
        self.recorder = MultiSensorRecorder(self.node, output_base_dir=base_data_dir)
        sample_interval = self.config['recording'].get('sample_interval_sec', 300)
        stream_fps = self.config['recording'].get('web_stream_hz', 5)

        self.zed_front = self.recorder.add_zed_camera(
            serial_number=48335070,
            name="front",
            socketio=self.socketio,
            sample_interval_sec=sample_interval,
            always_stream=True,
            stream_fps=stream_fps,
        )
        self.zed_back = self.recorder.add_zed_camera(
            serial_number=49537850,
            name="back",
            socketio=self.socketio,
            sample_interval_sec=sample_interval,
            always_stream=True,
            stream_fps=stream_fps,
        )
        print("ZED cameras initialized")

        self.server_to_panel = ServerToPanel(self.socketio, self.config)
        self.map_controller  = MapCreationController(self.cmd_vel_pub, self.config)
        self.path_planner    = PathPlanner(self.config)

        # Mapper: emits robot_pose at ~10 Hz and map_updated at 1 Hz directly via socketio
        self.auto_controller = AutonomousController(
            self.cmd_vel_pub, self.socketio, self.config
        )
        print("All modules initialized")

        self.is_recording        = False
        self.current_session_dir = None

        self.register_routes()
        self.register_socketio_events()

        signal.signal(signal.SIGINT,  self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        atexit.register(self.cleanup)

    # ── Routes ──────────────────────────────────────────────────────────────
    # HTTP routes for /front_rgb  /back_rgb  /map_latest are all removed
    # Image delivery is handled exclusively via SocketIO push
    def register_routes(self):
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

        @self.app.route('/map_latest.png')
        def serve_map():
            """Serve map image via HTTP - avoids sending large blobs through socket.io polling"""
            import os
            map_path = os.path.join(
                os.path.expanduser(self.config['paths']['map_dir']),
                'map_latest.png'
            )
            if not os.path.exists(map_path):
                return ('Map not found', 404)
            return send_file(map_path, mimetype='image/png',
                             max_age=0,
                             conditional=False)

    # ── SocketIO Events ──────────────────────────────────────────────────────
    def register_socketio_events(self):

        @self.socketio.on('connect')
        def handle_connect():
            print(f"✅ Client connected: {request.sid}")
            self.map_controller.should_update_twist = True
            # Send map immediately on connect
            self.server_to_panel.send_map_update(force=True)
            # Re-emit cached pose so robot marker appears immediately after refresh
            pose = self.auto_controller.get_current_pose()
            if pose:
                self.socketio.emit('robot_pose', pose)
            # Sync recording state to newly connected client
            if self.is_recording:
                self.socketio.emit('recording_status', {
                    'active':  True,
                    'dirname': self.current_session_dir or '',
                })

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
            waypoints = data.get('waypoints', [])
            self.auto_controller.start(waypoints)

        @self.socketio.on('stop_autonomous')
        def handle_stop_autonomous():
            self.auto_controller.stop()

        # ★ Streaming toggle event (client checkbox → server)
        @self.socketio.on('toggle_streaming')
        def handle_toggle_streaming(data):
            enabled = data.get('enabled', True)
            self.zed_front.set_streaming(enabled)
            self.zed_back.set_streaming(enabled)
            print(f"Streaming {'enabled' if enabled else 'disabled'}")

        @self.socketio.on('start_recording')
        def handle_start_recording(dirname):
            print(f"🔴 start_recording: {dirname}")
            if self.is_recording:
                self.recorder.stop_recording()
                self.is_recording = False
            if not dirname:
                return
            from datetime import datetime
            timestamp    = datetime.now().strftime("%Y%m%d_%H%M")
            session_name = f"{dirname}/{timestamp}"
            try:
                self.current_session_dir = self.recorder.create_session(session_name)
                self.recorder.start_recording()
                self.is_recording = True
                print(f"✅ Recording: {self.current_session_dir}")
            except Exception as e:
                print(f"❌ Failed to start recording: {e}")
                self.is_recording = False

        @self.socketio.on('stop_recording')
        def handle_stop_recording():
            print("⏹️ Stopping recording...")
            if not self.is_recording:
                return
            try:
                self.recorder.stop_recording()
                self.is_recording = False
                self.current_session_dir = None
                print("✅ Recording stopped")
            except Exception as e:
                print(f"❌ Error stopping recording: {e}")

    def signal_handler(self, sig, frame):
        print(f"\nReceived signal {sig}, shutting down...")
        self.shutdown_flag.set()
        self.cleanup()
        sys.exit(0)

    def ros_spin(self):
        while rclpy.ok() and not self.shutdown_flag.is_set():
            rclpy.spin_once(self.recorder.ros_node, timeout_sec=0.05)
            rclpy.spin_once(self.auto_controller, timeout_sec=0.05)

    def cleanup(self):
        if self.shutdown_flag.is_set():
            return
        print("\n🧹 Cleaning up...")
        self.shutdown_flag.set()
        try:
            self.map_controller.clear_keys_and_stop()
        except Exception as e:
            print(f"Error stopping robot: {e}")
        try:
            if self.auto_controller.is_active():
                self.auto_controller.stop()
        except Exception as e:
            print(f"Error stopping autonomous mode: {e}")
        try:
            if self.is_recording:
                self.recorder.stop_recording()
                self.is_recording = False
        except Exception as e:
            print(f"Error stopping recording: {e}")
        try:
            self.recorder.shutdown()
        except Exception as e:
            print(f"Error during recorder shutdown: {e}")
        print("✅ Cleanup complete")

    def run(self):
        threading.Thread(
            target=self.server_to_panel.monitor_loop,
            args=(self.map_controller.get_linear_speed,),
            daemon=True
        ).start()

        threading.Thread(target=self.ros_spin, daemon=True).start()

        threading.Thread(
            target=self.map_controller.update_loop,
            args=(self.auto_controller.is_active,),
            daemon=True
        ).start()

        os.makedirs(os.path.expanduser(self.config['paths']['data_dir']), exist_ok=True)

        print(f"🚀 Control server on port {self.config['server']['port']}")
        try:
            self.socketio.run(
                self.app,
                host=self.config['server']['host'],
                port=self.config['server']['port'],
                allow_unsafe_werkzeug=True,
                use_reloader=False,
                # threaded=True is implicit when async_mode='threading'
            )
        except KeyboardInterrupt:
            print("\n⚠️ Keyboard interrupt")
        finally:
            self.cleanup()


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')
    server = ControlServer('control_config.yaml')
    server.run()