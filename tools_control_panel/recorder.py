import os
import time
import base64
import threading
import pyzed.sl as sl
import cv2
import subprocess
import signal
import numpy as np
from datetime import datetime

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import Image

stop_event = threading.Event()

PUBLISH_HZ = 3.0


def _make_qos_sensor():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT
    )


def _np_to_img_msg(np_img, encoding, frame_id, stamp_sec, stamp_nsec):
    msg = Image()
    msg.header = Header()
    msg.header.stamp.sec      = int(stamp_sec)
    msg.header.stamp.nanosec  = int(stamp_nsec)
    msg.header.frame_id       = frame_id
    h, w = np_img.shape[:2]
    msg.height     = h
    msg.width      = w
    msg.encoding   = encoding
    msg.is_bigendian = False
    if encoding == "rgb8":
        if np_img.dtype != np.uint8:
            np_img = np_img.astype(np.uint8, copy=False)
        msg.step = w * 3
        msg.data = np_img.tobytes()
    else:
        raise ValueError(f"Unsupported encoding: {encoding}")
    return msg


class ZEDSVORecorder(threading.Thread):
    def __init__(self, serial_number, name, output_dir, ros_node,
                 socketio=None, sample_interval_sec=300,
                 always_stream=False, publish_hz=PUBLISH_HZ,
                 stream_fps=5,        # ★ Web streaming FPS (independent from ROS publish Hz)
                 frame_id=None):
        super().__init__(name=name)
        self.serial   = serial_number
        self.name     = name
        self.output_dir        = output_dir
        self.ros_node          = ros_node
        self.socketio          = socketio
        self.running           = True
        self.recording         = False
        self.always_stream     = always_stream
        self.sample_interval_sec = sample_interval_sec
        self.publish_hz        = float(publish_hz)
        self.publish_period    = 1.0 / self.publish_hz
        self.frame_id          = frame_id or f"zed_{name}"

        # ★ Web streaming interval (independent from publish_hz)
        self.stream_fps    = float(stream_fps)
        self.stream_period = 1.0 / self.stream_fps
        self._streaming_enabled = True
        self._stream_lock       = threading.Lock()

        # Web stream resolution: fixed height, width scaled to preserve aspect ratio
        self.web_rgb_height = 200         # Must match --rgb-h in styles.css
        # ROS publish resolution (keep original scale)
        self.downscale_factor = 0.2

        self.latest_frame = None
        self.frame_lock   = threading.Lock()

        self.rgb_topic = f"/zed/rgb_{self.name.lower()}"
        self.rgb_pub   = self.ros_node.create_publisher(
            Image, self.rgb_topic, _make_qos_sensor()
        )

        try:
            self.cam  = sl.Camera()
            init      = sl.InitParameters()
            init.set_from_serial_number(serial_number)
            init.depth_mode        = sl.DEPTH_MODE.ULTRA
            init.camera_resolution = sl.RESOLUTION.HD1080
            init.camera_fps        = 30
            init.sdk_verbose       = False

            status = self.cam.open(init)
            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"ZED {serial_number} Open Failed: {status}")

            self.image_zed = sl.Mat()
            self.runtime   = sl.RuntimeParameters()
        except Exception as e:
            print(f"[{self.name}] Camera init error: {e}")
            self.cam = None

    # ── Streaming Toggle ──────────────────────────────────────────────────

    def set_streaming(self, enabled: bool):
        with self._stream_lock:
            self._streaming_enabled = enabled

    def get_latest_frame(self):
        with self.frame_lock:
            return self.latest_frame

    # ── Main Loop ─────────────────────────────────────────────────────────

    def run(self):
        if self.cam is None:
            print(f"[{self.name}] Camera not available, thread exiting")
            return

        print(f"[{self.name}] Thread started. ROS:{self.publish_hz}Hz  Stream:{self.stream_fps}Hz")
        next_pub_time    = time.time()
        next_stream_time = time.time()
        next_sample_time = time.time()

        while self.running and not stop_event.is_set():
            if self.cam.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.01)
                continue

            now = time.time()

            # ── ROS publish (lower Hz, preserves existing logic) ───────────
            if now >= next_pub_time:
                self.cam.retrieve_image(self.image_zed, sl.VIEW.LEFT)
                rgba_np    = self.image_zed.get_data()
                h, w       = rgba_np.shape[:2]
                small_rgba = cv2.resize(rgba_np,
                                        (int(w * self.downscale_factor),
                                         int(h * self.downscale_factor)),
                                        interpolation=cv2.INTER_NEAREST)
                rgb_small  = cv2.cvtColor(small_rgba, cv2.COLOR_RGBA2RGB)

                try:
                    zed_ts = self.cam.get_timestamp(sl.TIME_REFERENCE.IMAGE)
                    ts_ns  = zed_ts.get_nanoseconds()
                    msg    = _np_to_img_msg(
                        rgb_small, "rgb8", self.frame_id,
                        ts_ns // 1_000_000_000, ts_ns % 1_000_000_000
                    )
                    self.rgb_pub.publish(msg)
                except Exception as e:
                    print(f"[{self.name}] ROS Pub Error: {e}")

                next_pub_time = now + self.publish_period

            # ── Web streaming push (independent Hz) ───────────────────────
            if now >= next_stream_time:
                with self._stream_lock:
                    streaming_on = self._streaming_enabled

                if self.always_stream and self.socketio and streaming_on:
                    # retrieve_image is called again since this runs on an independent interval
                    self.cam.retrieve_image(self.image_zed, sl.VIEW.LEFT)
                    rgba_np = self.image_zed.get_data()
                    h, w    = rgba_np.shape[:2]

                    # ★ Resize to fixed height, preserve aspect ratio
                    h_orig, w_orig = rgba_np.shape[:2]
                    target_h = self.web_rgb_height
                    target_w = int(w_orig * target_h / h_orig)
                    small = cv2.resize(rgba_np, (target_w, target_h), interpolation=cv2.INTER_NEAREST)
                    rgb   = cv2.cvtColor(small, cv2.COLOR_RGBA2RGB)

                    # ★ JPEG compress at quality 40
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]
                    _, buffer    = cv2.imencode('.jpg', rgb, encode_param)
                    b64_image    = base64.b64encode(buffer).decode('utf-8')

                    # ★ SocketIO push (replaces HTTP polling)
                    event_name = f"{self.name}_frame"   # 'front_frame' or 'back_frame'
                    self.socketio.emit(event_name, {'data': b64_image})

                    # Keep get_latest_frame() compatible for legacy use
                    with self.frame_lock:
                        self.latest_frame = buffer.tobytes()

                next_stream_time = now + self.stream_period

            # ── Save sample image ──────────────────────────────────────────
            if self.recording and now >= next_sample_time:
                timestamp_str = datetime.now().strftime("%m%d_%H%M")
                self.save_sample(timestamp_str)
                next_sample_time = now + self.sample_interval_sec

        self.cam.close()

    def stop(self):
        self.running = False
        if self.is_alive():
            self.join()
        if self.cam:
            self.cam.close()

    def start_recording(self, output_dir):
        self.output_dir  = output_dir
        self.svo_path    = os.path.join(output_dir, f"rgbd_{self.name}.svo2")
        self.samples_dir = os.path.join(output_dir, "samples")
        os.makedirs(self.samples_dir, exist_ok=True)

        rec_params = sl.RecordingParameters(self.svo_path, sl.SVO_COMPRESSION_MODE.H265)
        rec_params.transcode_streaming_input = True

        if self.cam.enable_recording(rec_params) != sl.ERROR_CODE.SUCCESS:
            print(f"[{self.name}] Failed to start SVO recording")
        else:
            self.recording        = True
            self.last_sample_time = time.time()

    def stop_recording(self):
        self.recording = False
        self.cam.disable_recording()

    def save_sample(self, timestamp_str):
        self.cam.retrieve_image(self.image_zed, sl.VIEW.LEFT)
        img  = cv2.cvtColor(self.image_zed.get_data(), cv2.COLOR_RGBA2RGB)
        path = os.path.join(self.samples_dir, f"{timestamp_str}_{self.name}.jpg")
        cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 85])


class MultiSensorRecorder:
    def __init__(self, ros_node, output_base_dir="./data"):
        self.output_base_dir     = output_base_dir
        self.current_session_dir = None
        self.ros_node            = ros_node
        self.zed_recorders       = {}
        self.rosbag_process      = None

    def create_session(self, session_name=None):
        if session_name is None:
            session_name = datetime.now().strftime("%Y%m%d_%H%M")
        self.current_session_dir = os.path.join(self.output_base_dir, session_name)
        os.makedirs(self.current_session_dir, exist_ok=True)
        print(f"\n[Recorder] Session created: {self.current_session_dir}\n")
        return self.current_session_dir

    def add_zed_camera(self, serial_number, name, **kwargs):
        recorder = ZEDSVORecorder(
            serial_number=serial_number,
            name=name,
            output_dir=self.current_session_dir,
            ros_node=self.ros_node,
            **kwargs,
        )
        self.zed_recorders[name] = recorder
        recorder.start()
        return recorder

    def start_recording(self):
        rosbag_dir = os.path.join(self.current_session_dir, "rosbag")
        self.rosbag_process = subprocess.Popen([
            "ros2", "bag", "record",
            "-o", rosbag_dir,
            "-e", "/livox/.*|/zed/.*|/ground_from_map|/gps_trajectory_marker|/gps/fix"
        ])
        print("\n[Recorder] Rosbag recording started\n")

    def stop_recording(self):
        if self.rosbag_process:
            self.rosbag_process.send_signal(signal.SIGINT)
            try:
                self.rosbag_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.rosbag_process.kill()
            self.rosbag_process = None
        print("\n[Recorder] Recording stopped\n")

    def shutdown(self):
        if self.rosbag_process:
            self.rosbag_process.send_signal(signal.SIGINT)
            try:
                self.rosbag_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.rosbag_process.kill()
            self.rosbag_process = None
        for recorder in self.zed_recorders.values():
            recorder.stop()