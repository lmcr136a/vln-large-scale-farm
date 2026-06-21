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


def _is_explicit_serial(s):
    return isinstance(s, int) or (isinstance(s, str) and s.strip().isdigit())


def resolve_zed_serials(cameras):
    """Resolve camera serials against the connected ZED devices.

    cameras: list of {"serial": <int|"auto"|None>, "name": <str>, ...}.
    Explicit serials are kept; cameras with a non-numeric serial (e.g. "auto")
    are filled, in list order, from connected devices not already claimed.
    Mirrors the KNOWN + auto-detect approach in snapshot_all_zeds.py.
    """
    try:
        devices   = sl.Camera.get_device_list()
        connected = [int(d.serial_number) for d in devices]
    except Exception as e:
        print(f"[recorder] get_device_list failed: {e}")
        connected = []
    print(f"[recorder] Connected ZED serials: {connected}")

    explicit = {int(c["serial"]) for c in cameras if _is_explicit_serial(c.get("serial"))}
    leftover = [s for s in connected if s not in explicit]

    resolved = []
    li = 0
    for c in cameras:
        s = c.get("serial")
        if _is_explicit_serial(s):
            resolved.append({**c, "serial": int(s)})
        elif li < len(leftover):
            resolved.append({**c, "serial": leftover[li]})
            print(f"[recorder] auto-assigned serial {leftover[li]} -> '{c.get('name')}'")
            li += 1
        else:
            print(f"[recorder] no serial available for '{c.get('name')}', skipping")
    return resolved


class ZEDSVORecorder(threading.Thread):
    def __init__(self, serial_number, name, output_dir,
                 ros_node=None,
                 socketio=None, sample_interval_sec=300,
                 always_stream=False,
                 stream_fps=STREAM_FPS,
                 publish_hz=None,
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
        self.web_rgb_height      = 160
        self.latest_frame        = None
        self.frame_lock          = threading.Lock()
        self._cam_closed         = False  # guard against double-close
        self.cam                 = None  # opened in run(), not here, so a slow/failing
                                          # open() can't block the other recorders or the
                                          # main thread's socket.io connection

    def _open_cam(self):
        try:
            self.cam  = sl.Camera()
            init      = sl.InitParameters()
            init.set_from_serial_number(self.serial)
            init.depth_mode        = sl.DEPTH_MODE.NONE
            init.camera_resolution = sl.RESOLUTION.HD1080
            init.camera_fps        = 30
            init.sdk_verbose       = False
            init.sensors_required  = False

            # open() now runs on this recorder's own thread (not main), and Python's
            # signal module only allows signal.signal() calls from the main thread.
            import signal as _signal
            on_main_thread = threading.current_thread() is threading.main_thread()
            if on_main_thread:
                _prev_sigint  = _signal.getsignal(_signal.SIGINT)
                _prev_sigterm = _signal.getsignal(_signal.SIGTERM)

            status = self.cam.open(init)

            if on_main_thread:
                _signal.signal(_signal.SIGINT,  _prev_sigint)
                _signal.signal(_signal.SIGTERM, _prev_sigterm)

            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"ZED {self.serial} open failed: {status}")

            self.image_zed = sl.Mat()
            self.runtime   = sl.RuntimeParameters()
            return True
        except Exception as e:
            print(f"[{self.name}] Camera init error: {e}")
            self.cam = None
            return False

    def _close_cam(self):
        if self.cam and not self._cam_closed:
            self._cam_closed = True
            self.cam.close()

    def set_streaming(self, enabled: bool):
        with self._stream_lock:
            self._streaming_enabled = enabled

    def get_latest_frame(self):
        with self.frame_lock:
            return self.latest_frame

    def run(self):
        if not self._open_cam():
            print(f"[{self.name}] Camera not available, thread exiting")
            return

        print(f"[{self.name}] Thread started. Stream:{self.stream_fps}Hz")
        next_stream_time = time.time()
        next_sample_time = time.time()

        try:
            while self.running and not stop_event.is_set():
                if self.cam.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
                    time.sleep(0.01)
                    continue

                now = time.time()

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

                if self.recording and now >= next_sample_time:
                    self.save_sample(datetime.now().strftime("%m%d_%H%M"))
                    next_sample_time = now + self.sample_interval_sec
        finally:
            self._close_cam()  # always closed exactly once here

    def stop(self):
        self.running = False
        if self.is_alive():
            self.join(timeout=5.0)
        # _close_cam is a no-op if run() already closed it

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
        timeout = time.time() + 15.0
        while time.time() < timeout:
            if recorder.get_latest_frame() is not None:
                break
            time.sleep(0.5)
        else:
            print(f"[{name}] Warning: no frame received within 15s")
        return recorder

    def start_recording(self, output_dir=None):
        if output_dir:
            self.current_session_dir = output_dir
        for recorder in self.zed_recorders.values():
            recorder.start_recording(self.current_session_dir)

    def stop_recording(self):
        for recorder in self.zed_recorders.values():
            recorder.stop_recording()

    def shutdown(self):
        stop_event.set()
        for recorder in self.zed_recorders.values():
            recorder.stop()


def _make_signal_handler(recorder: MultiSensorRecorder):
    def handler(sig, frame):
        print("\n[MultiSensorRecorder] Signal received, shutting down...")
        recorder.shutdown()
    return handler


def setup_signal_handlers(recorder: MultiSensorRecorder):
    signal.signal(signal.SIGINT,  _make_signal_handler(recorder))
    signal.signal(signal.SIGTERM, _make_signal_handler(recorder))