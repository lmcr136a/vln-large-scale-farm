#!/usr/bin/env python3
"""
Control Server - Main Entry Point
"""
from dataclasses import replace as dc_replace
import json
import os
import sys
import yaml
import threading
import signal
import atexit
import time

from flask import Flask, send_file, request
from flask_socketio import SocketIO

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String as RosString

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools_safety_nav.agro_nav.fusion import CameraIntrinsics, detections_to_observations
from tools_safety_nav.agro_nav.policy import SemanticReflexController
from tools_safety_nav.agro_nav.types import BoundingBox2D, SemanticDetection, observations_to_payload
from server_to_panel import ServerToPanel
from map_creation import MapCreationController
from get_path import PathPlanner
from autonomous_mode import AutonomousController
from recorder import MultiSensorRecorder
from path_utils import resolve_configured_path
import logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)


class ControlServer:
    def __init__(self, config_path='control_config.yaml'):
        self.shutdown_flag = threading.Event()
        self.semantic_thread = None
        self.semantic_stop = threading.Event()
        self.semantic_pub = None
        self.semantic_model = None
        self.semantic_device = None
        self.semantic_period = 0.25
        self.semantic_hold_sec = 0.6
        self._last_front_semantics = None
        self._semantic_tracks = []
        self._semantic_track_seq = 0
        self._last_semantic_packet_ts = None
        self._cleanup_started = False

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        print("Configuration loaded")
        semantics_cfg = self.config.get('semantics', {}) or {}
        self.semantic_hold_sec = max(float(semantics_cfg.get('hold_sec', 0.6)), 0.0)
        self.semantic_match_distance = max(float(semantics_cfg.get('match_distance', 0.18)), 0.05)
        self.semantic_classifier = SemanticReflexController(
            nominal_linear_speed=float(self.config.get('robot', {}).get('max_linear_speed', 1.5)),
        )

        self.map_dir = resolve_configured_path(
            self.config['paths'].get('map_dir'),
            os.path.join('tools_control_panel', 'output_glim'),
        )
        self.data_dir = resolve_configured_path(
            self.config['paths'].get('data_dir'),
            'data',
        )
        self.config['paths']['map_dir'] = self.map_dir
        self.config['paths']['data_dir'] = self.data_dir
        print(f"Resolved map dir: {self.map_dir}")
        print(f"Resolved data dir: {self.data_dir}")

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
        self.recorder = MultiSensorRecorder(self.node, output_base_dir=self.data_dir)
        sample_interval = self.config['recording'].get('sample_interval_sec', 300)
        stream_fps = self.config['recording'].get('web_stream_hz', 5)

        self.zed_front = self.recorder.add_zed_camera(
            serial_number=48335070,
            name="front",
            socketio=self.socketio,
            sample_interval_sec=sample_interval,
            always_stream=True,
            stream_fps=stream_fps,
            semantic_capture_hz=float(semantics_cfg.get('capture_hz', max(stream_fps, 10))),
            semantic_input_height=int(semantics_cfg.get('capture_height', 360)),
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

        self.node.create_subscription(
            RosString, '/agro_nav/decision', self._decision_cb, 20
        )
        self.semantic_pub = self.node.create_publisher(RosString, '/semantic_observations', 20)
        self._last_decision_msg = None

        self.is_recording        = False
        self.current_session_dir = None

        self.register_routes()
        self.register_socketio_events()

        signal.signal(signal.SIGINT,  self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        atexit.register(self.cleanup)

    def _start_semantics(self):
        semantics_cfg = self.config.get('semantics', {}) or {}
        if not semantics_cfg.get('enabled', True):
            print("Embedded semantics disabled in config")
            return
        if self.semantic_thread is not None and self.semantic_thread.is_alive():
            return
        self.semantic_stop.clear()
        self.semantic_period = 1.0 / max(float(semantics_cfg.get('inference_hz', 4.0)), 0.5)
        self.semantic_thread = threading.Thread(target=self._semantic_loop, daemon=True)
        self.semantic_thread.start()
        print("Embedded semantics started")

    def _stop_semantics(self):
        self.semantic_stop.set()
        if self.semantic_thread is not None and self.semantic_thread.is_alive():
            self.semantic_thread.join(timeout=5)
        self.semantic_thread = None

    def _resolve_semantic_model_path(self):
        semantics_cfg = self.config.get('semantics', {}) or {}
        configured = os.path.expanduser(str(semantics_cfg.get('model_path', '')).strip())
        candidates = [
            configured,
            os.path.join(REPO_ROOT, 'models', 'yolov8n-seg.pt'),
            os.path.expanduser('~/box/vln-large-scale-farm/models/yolov8n-seg.pt'),
        ]
        for candidate in candidates:
            if candidate and os.path.isfile(candidate):
                return candidate
        return None

    def _load_semantic_model(self):
        if self.semantic_model is not None:
            return self.semantic_model
        model_path = self._resolve_semantic_model_path()
        if not model_path:
            raise FileNotFoundError('No YOLO semantic model found')
        from ultralytics import YOLO
        semantics_cfg = self.config.get('semantics', {}) or {}
        self.semantic_device = str(semantics_cfg.get('device', 'cuda:0')).strip() or 'cpu'
        self.semantic_model = YOLO(model_path)
        print(f"Embedded semantics loaded model: {model_path}")
        return self.semantic_model

    def _predict_semantics(self, rgb_image):
        model = self._load_semantic_model()
        semantics_cfg = self.config.get('semantics', {}) or {}
        confidence_threshold = float(semantics_cfg.get('confidence_threshold', 0.35))
        try:
            return model.predict(
                source=rgb_image,
                conf=confidence_threshold,
                device=self.semantic_device,
                verbose=False,
            )
        except Exception as e:
            if self.semantic_device != 'cpu':
                print(f"Semantic inference failed on {self.semantic_device}, retrying on cpu: {e}")
                self.semantic_device = 'cpu'
                return model.predict(
                    source=rgb_image,
                    conf=confidence_threshold,
                    device='cpu',
                    verbose=False,
                )
            raise

    def _observation_center(self, observation, width, height):
        bbox = observation.bbox
        if bbox is None:
            return None
        return (
            bbox.center_x / max(float(width), 1.0),
            bbox.center_y / max(float(height), 1.0),
        )

    def _classify_observation_compliance(self, observation):
        if observation is None:
            return "none"
        profile = self.semantic_classifier.risk_profiles.get(
            observation.label,
            self.semantic_classifier.risk_profiles["unknown"],
        )
        if profile.family == "compliant":
            return "compliant" if self.semantic_classifier._passes_compliance_check(observation) else "non_compliant"
        if profile.family == "semi_compliant":
            return "semi_compliant"
        return "non_compliant"

    def _refresh_semantic_tracks(self, observations, frame_width, frame_height, now):
        surviving_tracks = []
        for track in self._semantic_tracks:
            age = now - track['last_seen']
            if age <= self.semantic_hold_sec:
                track['matched'] = False
                track['age_sec'] = age
                surviving_tracks.append(track)

        for observation in observations:
            center = self._observation_center(observation, frame_width, frame_height)
            if center is None:
                continue

            best_track = None
            best_distance = None
            for track in surviving_tracks:
                if track['matched'] or track['label'] != observation.label:
                    continue
                distance = abs(track['cx'] - center[0]) + abs(track['cy'] - center[1])
                if distance > self.semantic_match_distance:
                    continue
                if best_distance is None or distance < best_distance:
                    best_track = track
                    best_distance = distance

            if best_track is None:
                self._semantic_track_seq += 1
                best_track = {
                    'id': self._semantic_track_seq,
                    'label': observation.label,
                }
                surviving_tracks.append(best_track)

            best_track['matched'] = True
            best_track['last_seen'] = now
            best_track['age_sec'] = 0.0
            best_track['cx'], best_track['cy'] = center
            best_track['observation'] = observation
            best_track['compliance'] = self._classify_observation_compliance(observation)

        serialized = []
        published_observations = []
        next_tracks = []
        for track in surviving_tracks:
            observation = track.get('observation')
            if observation is None or observation.bbox is None:
                continue

            age_sec = max(0.0, now - track['last_seen'])
            if age_sec > self.semantic_hold_sec:
                continue

            confidence = observation.confidence
            if age_sec > 0.0 and self.semantic_hold_sec > 0.0:
                decay = max(0.55, 1.0 - (age_sec / self.semantic_hold_sec) * 0.45)
                confidence *= decay
            display_observation = dc_replace(observation, confidence=float(confidence))
            published_observations.append(display_observation)
            serialized.append({
                'id': track['id'],
                'label': display_observation.label,
                'confidence': round(display_observation.confidence, 3),
                'distance': round(display_observation.distance, 3),
                'bearing_deg': round(display_observation.bearing_deg, 2),
                'in_path': bool(display_observation.in_path),
                'source': display_observation.source,
                'compliance': track.get('compliance', self._classify_observation_compliance(display_observation)),
                'persisted': age_sec > 0.0,
                'age_ms': round(age_sec * 1000.0, 1),
                'bbox': {
                    'x_min': round(display_observation.bbox.x_min, 1),
                    'y_min': round(display_observation.bbox.y_min, 1),
                    'x_max': round(display_observation.bbox.x_max, 1),
                    'y_max': round(display_observation.bbox.y_max, 1),
                },
            })
            track['matched'] = False
            next_tracks.append(track)

        self._semantic_tracks = next_tracks
        return published_observations, serialized

    def _publish_front_semantics(self, frame_width, frame_height, detections, packet_timestamp_ns):
        payload = {
            'frame_width': int(frame_width),
            'frame_height': int(frame_height),
            'timestamp_ns': int(packet_timestamp_ns),
            'detections': detections,
        }
        self._last_front_semantics = payload
        self.socketio.emit('front_semantics', payload, namespace='/')

    def _semantic_loop(self):
        last_error = None
        next_run = time.monotonic()
        while not self.shutdown_flag.is_set() and not self.semantic_stop.is_set():
            now = time.monotonic()
            if now < next_run:
                time.sleep(min(next_run - now, 0.02))
                continue
            next_run = max(next_run + self.semantic_period, now + 0.001)
            try:
                packet = self.zed_front.get_latest_semantic_packet()
                if not packet:
                    continue

                packet_timestamp_ns = int(packet.get('timestamp_ns', 0))
                if packet_timestamp_ns and packet_timestamp_ns == self._last_semantic_packet_ts:
                    continue
                self._last_semantic_packet_ts = packet_timestamp_ns

                rgb = packet['rgb']
                results = self._predict_semantics(rgb)
                detections = []
                for result in results:
                    names = result.names
                    if result.boxes is None:
                        continue
                    for box in result.boxes:
                        confidence = float(box.conf[0].item())
                        class_id = int(box.cls[0].item())
                        x_min, y_min, x_max, y_max = [float(item) for item in box.xyxy[0].tolist()]
                        detections.append(
                            SemanticDetection(
                                label=str(names[class_id]),
                                confidence=confidence,
                                bbox=BoundingBox2D(
                                    x_min=x_min,
                                    y_min=y_min,
                                    x_max=x_max,
                                    y_max=y_max,
                                ),
                            )
                        )

                intrinsics_dict = packet.get('intrinsics') or {}
                height, width = rgb.shape[:2]
                intrinsics = CameraIntrinsics(
                    fx=float(intrinsics_dict.get('fx', width * 0.5)),
                    fy=float(intrinsics_dict.get('fy', height * 0.9)),
                    cx=float(intrinsics_dict.get('cx', width * 0.5)),
                    cy=float(intrinsics_dict.get('cy', height * 0.5)),
                    width=int(width),
                    height=int(height),
                )
                observations = detections_to_observations(
                    detections=detections,
                    depth_image=packet.get('depth'),
                    intrinsics=intrinsics,
                    lidar_ranges=None,
                    lidar_angle_min=0.0,
                    lidar_angle_increment=0.0,
                )
                display_observations, overlay_detections = self._refresh_semantic_tracks(
                    observations=observations,
                    frame_width=width,
                    frame_height=height,
                    now=time.monotonic(),
                )
                self.semantic_pub.publish(
                    RosString(data=json.dumps(observations_to_payload(display_observations)))
                )
                self._publish_front_semantics(
                    frame_width=width,
                    frame_height=height,
                    detections=overlay_detections,
                    packet_timestamp_ns=packet_timestamp_ns,
                )
                last_error = None
            except Exception as e:
                if str(e) != last_error:
                    print(f"Semantic loop error: {e}")
                    last_error = str(e)

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
            map_path = os.path.join(self.map_dir, 'map_latest.png')
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
            self.server_to_panel.send_map_update(force=True)
            pass  # robot_pose emitted directly by autonomous_mode
            if self._last_decision_msg is not None:
                self.socketio.emit('obstacle_decision', self._last_decision_msg, namespace='/')
            if self._last_front_semantics is not None:
                self.socketio.emit('front_semantics', self._last_front_semantics, namespace='/')
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

    def _decision_cb(self, msg: RosString) -> None:
        self._last_decision_msg = msg.data
        self.socketio.emit('obstacle_decision', msg.data, namespace='/')

    def ros_spin(self):
        while rclpy.ok() and not self.shutdown_flag.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.02)
            rclpy.spin_once(self.recorder.ros_node, timeout_sec=0.05)
            rclpy.spin_once(self.auto_controller, timeout_sec=0.05)

    def cleanup(self):
        if self._cleanup_started:
            return
        self._cleanup_started = True
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
        try:
            self._stop_semantics()
        except Exception as e:
            print(f"Error stopping semantics: {e}")
        print("✅ Cleanup complete")

    def run(self):
        threading.Thread(
            target=self.server_to_panel.monitor_loop,
            args=(self.map_controller.get_linear_speed,),
            daemon=True
        ).start()

        threading.Thread(target=self.ros_spin, daemon=True).start()
        self._start_semantics()

        threading.Thread(
            target=self.map_controller.update_loop,
            args=(self.auto_controller.is_active,),
            daemon=True
        ).start()

        os.makedirs(self.data_dir, exist_ok=True)

        print(f"🚀 Control server on port {self.config['server']['port']}")
        try:
            self.socketio.run(
                self.app,
                host=self.config['server']['host'],
                port=self.config['server']['port'],
                allow_unsafe_werkzeug=True,
                use_reloader=False,
                log_output=False,
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
