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

stop_event = threading.Event()

STREAM_FPS = 5.0


class ZEDSVORecorder(threading.Thread):
    def __init__(self, serial_number, name, output_dir,
                 ros_node=None,
                 socketio=None, sample_interval_sec=300,
                 always_stream=False,
                 stream_fps=STREAM_FPS,
                 publish_hz=None,   # ignored, kept for API compat
                 frame_id=None):
        super().__init__(name=name, daemon=True)
        self.serial              = serial_number
        self.name                = name
        self.output_dir          = output_dir
        self.socketio            = socketio
        self.running             = True
        self.recording           = False
        self.always_stream       = always_stream
        self.sample_interval_sec = sample_interval_sec
        self.stream_fps          = float(stream_fps)
        self.stream_period       = 1.0 / self.stream_fps
        self._streaming_enabled  = True
        self._stream_lock        = threading.Lock()
        self.web_rgb_height      = 200
        self.latest_frame        = None
        self.frame_lock          = threading.Lock()

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
                raise RuntimeError(f"ZED {serial_number} open failed: {status}")

            self.image_zed = sl.Mat()
            self.runtime   = sl.RuntimeParameters()
        except Exception as e:
            print(f"[{self.name}] Camera init error: {e}")
            self.cam = None

    def set_streaming(self, enabled: bool):
        with self._stream_lock:
            self._streaming_enabled = enabled

    def get_latest_frame(self):
        with self.frame_lock:
            return self.latest_frame

    def run(self):
        if self.cam is None:
            print(f"[{self.name}] Camera not available, thread exiting")
            return

        print(f"[{self.name}] Thread started. Stream:{self.stream_fps}Hz")
        next_stream_time = time.time()
        next_sample_time = time.time()

        while self.running and not stop_event.is_set():
            if self.cam.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.01)
                continue

            now = time.time()

            # ── Web streaming ─────────────────────────────────────────────
            if now >= next_stream_time:
                with self._stream_lock:
                    streaming_on = self._streaming_enabled

                if self.always_stream and self.socketio and streaming_on:
                    self.cam.retrieve_image(self.image_zed, sl.VIEW.LEFT)
                    rgba_np = self.image_zed.get_data()
                    h_orig, w_orig = rgba_np.shape[:2]
                    target_h = self.web_rgb_height
                    target_w = int(w_orig * target_h / h_orig)
                    small    = cv2.resize(rgba_np, (target_w, target_h),
                                          interpolation=cv2.INTER_NEAREST)
                    rgb      = cv2.cvtColor(small, cv2.COLOR_RGBA2RGB)
                    _, buf   = cv2.imencode('.jpg', rgb,
                                            [int(cv2.IMWRITE_JPEG_QUALITY), 40])
                    b64      = base64.b64encode(buf).decode('utf-8')
                    self.socketio.emit(f"{self.name}_frame", {'data': b64})
                    with self.frame_lock:
                        self.latest_frame = buf.tobytes()

                next_stream_time = now + self.stream_period

            # ── Sample snapshot ───────────────────────────────────────────
            if self.recording and now >= next_sample_time:
                self.save_sample(datetime.now().strftime("%m%d_%H%M"))
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

        rec_params = sl.RecordingParameters(self.svo_path,
                                             sl.SVO_COMPRESSION_MODE.H265)
        rec_params.transcode_streaming_input = True

        if self.cam.enable_recording(rec_params) != sl.ERROR_CODE.SUCCESS:
            print(f"[{self.name}] Failed to start SVO2 recording")
        else:
            self.recording        = True
            self.last_sample_time = time.time()
            print(f"[{self.name}] SVO2 recording: {self.svo_path}")

    def stop_recording(self):
        self.recording = False
        self.cam.disable_recording()

    def save_sample(self, timestamp_str):
        self.cam.retrieve_image(self.image_zed, sl.VIEW.LEFT)
        img  = cv2.cvtColor(self.image_zed.get_data(), cv2.COLOR_RGBA2RGB)
        path = os.path.join(self.samples_dir, f"{timestamp_str}_{self.name}.jpg")
        cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 85])


class MultiSensorRecorder:
    def __init__(self, ros_node=None, output_base_dir="./data"):
        self.output_base_dir     = output_base_dir
        self.current_session_dir = None
        self.ros_node            = ros_node
        self.zed_recorders       = {}

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

    def start_recording(self, output_dir=None):
        if output_dir:
            self.current_session_dir = output_dir
        for name, recorder in self.zed_recorders.items():
            recorder.start_recording(self.current_session_dir)

    def stop_recording(self):
        for recorder in self.zed_recorders.values():
            recorder.stop_recording()

    def shutdown(self):
        for recorder in self.zed_recorders.values():
            recorder.stop()