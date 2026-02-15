import os
import time
import threading
import pyzed.sl as sl
import cv2
import numpy as np
from datetime import datetime

# ROS2 imports
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import Image

stop_event = threading.Event()

# ---- Publish rate (set 10.0, if too heavy -> 5.0) ----
PUBLISH_HZ = 3.0


def _make_qos_sensor():
    # Lighter QoS for high-bandwidth image topics
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT
    )


def _np_to_img_msg(np_img, encoding, frame_id, stamp_sec, stamp_nsec):
    """
    Create sensor_msgs/Image from numpy array.
    - np_img: HxWxC (uint8) for rgb
    - encoding: 'rgb8'
    """
    msg = Image()
    msg.header = Header()
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int(stamp_nsec)
    msg.header.frame_id = frame_id

    if np_img.ndim == 2:
        h, w = np_img.shape
        c = 1
    else:
        h, w, c = np_img.shape

    msg.height = h
    msg.width = w
    msg.encoding = encoding
    msg.is_bigendian = False

    if encoding == "rgb8":
        # Expect uint8 HxWx3
        if np_img.dtype != np.uint8:
            np_img = np_img.astype(np.uint8, copy=False)
        msg.step = w * 3
        msg.data = np_img.tobytes()
    else:
        raise ValueError(f"Unsupported encoding: {encoding}")

    return msg


class ZEDSVORecorder(threading.Thread):
    def __init__(self, serial_number, name, output_dir, ros_node, socketio=None, 
                 sample_interval_sec=300, always_stream=False, publish_hz=PUBLISH_HZ, frame_id=None):
        super().__init__(name=name)
        self.serial = serial_number
        self.name = name
        self.output_dir = output_dir
        self.ros_node = ros_node
        self.socketio = socketio
        self.running = True
        self.recording = False
        self.always_stream = always_stream
        self.sample_interval_sec = sample_interval_sec
        self.publish_hz = float(publish_hz)
        self.publish_period = 1.0 / self.publish_hz
        self.frame_id = frame_id or f"zed_{name}"

        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # Optimization: Downscale for ROS to 540p (0.5) or 360p (0.33)
        self.downscale_factor = 0.4 

        self.rgb_topic = f"/zed/rgb_{self.name.lower()}"
        self.rgb_pub = self.ros_node.create_publisher(Image, self.rgb_topic, _make_qos_sensor())

        self.cam = sl.Camera()
        init = sl.InitParameters()
        init.set_from_serial_number(serial_number)
        
        # PERFORMANCE FIX: NEURAL depth is too heavy for high-speed streaming.
        # Use ULTRA or PERFORMANCE if you aren't doing high-precision mapping.
        init.depth_mode = sl.DEPTH_MODE.ULTRA 
        
        init.camera_resolution = sl.RESOLUTION.HD1080 
        init.camera_fps = 30 # Set higher internal FPS
        init.sdk_verbose = False

        status = self.cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED {serial_number} Open Failed: {status}")

        # Pre-allocate sl.Mat to avoid memory fragmentation
        self.image_zed = sl.Mat()
        self.runtime = sl.RuntimeParameters()
        
    def run(self):
        print(f"[{self.name}] Thread started. Target: {self.publish_hz}Hz")
        next_pub_time = time.time()
        next_sample_time = time.time()

        while self.running and not stop_event.is_set():
            # grab() is the bottleneck. 
            if self.cam.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
                now = time.time()
                
                if now >= next_pub_time:
                    # 1. Retrieve directly
                    self.cam.retrieve_image(self.image_zed, sl.VIEW.LEFT)
                    
                    # 2. Fast conversion: Only process what is needed
                    # Note: get_data() is a heavy copy operation
                    rgba_np = self.image_zed.get_data()
                    
                    # 3. Resize first (reduces cvtColor workload by ~70%)
                    h, w = rgba_np.shape[:2]
                    small_rgba = cv2.resize(rgba_np, 
                                          (int(w * self.downscale_factor), int(h * self.downscale_factor)), 
                                          interpolation=cv2.INTER_NEAREST) # INTER_NEAREST is faster
                    
                    rgb_small = cv2.cvtColor(small_rgba, cv2.COLOR_RGBA2RGB)

                    # 4. Publish to ROS
                    try:
                        zed_ts = self.cam.get_timestamp(sl.TIME_REFERENCE.IMAGE)
                        ts_ns = zed_ts.get_nanoseconds()
                        
                        msg = _np_to_img_msg(
                            rgb_small, "rgb8", self.frame_id, 
                            ts_ns // 1_000_000_000, ts_ns % 1_000_000_000
                        )
                        self.rgb_pub.publish(msg)
                    except Exception as e:
                        print(f"[{self.name}] Pub Error: {e}")

                    # 5. Handle Web Stream (reuse resized image)
                    if self.always_stream and self.socketio:
                        _, buffer = cv2.imencode(".jpg", rgb_small, [cv2.IMWRITE_JPEG_QUALITY, 50])
                        with self.frame_lock:
                            self.latest_frame = buffer.tobytes()

                    next_pub_time = now + self.publish_period
                
                # 6. Save samples at specified interval
                if self.recording and now >= next_sample_time:
                    timestamp_str = datetime.now().strftime("%m%d_%H%M")
                    self.save_sample(timestamp_str)
                    next_sample_time = now + self.sample_interval_sec
            else:
                # Avoid busy-waiting if grab fails
                time.sleep(0.01)

        self.cam.close()

    def stop(self):
        self.running = False
        if self.is_alive():
            self.join()
        self.cam.close()

    def start_recording(self, output_dir):
        self.output_dir = output_dir
        self.svo_path = os.path.join(output_dir, f"rgbd_{self.name}.svo2")
        self.samples_dir = os.path.join(output_dir, "samples")
        os.makedirs(self.samples_dir, exist_ok=True)

        rec_params = sl.RecordingParameters(self.svo_path, sl.SVO_COMPRESSION_MODE.H265)
        rec_params.transcode_streaming_input = True
        
        if self.cam.enable_recording(rec_params) != sl.ERROR_CODE.SUCCESS:
            print(f"[{self.name}] Failed to start SVO recording")
        else:
            self.recording = True
            self.last_sample_time = time.time()

    def stop_recording(self):
        self.recording = False
        self.cam.disable_recording()

    def save_sample(self, timestamp_str):
        # Save RGB sample from current Mat
        self.cam.retrieve_image(self.image_zed, sl.VIEW.LEFT)
        img = cv2.cvtColor(self.image_zed.get_data(), cv2.COLOR_RGBA2RGB)
        path = os.path.join(self.samples_dir, f"{timestamp_str}_{self.name}.jpg")
        cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    
    def get_latest_frame(self):
        """Return the latest frame for web streaming"""
        with self.frame_lock:
            return self.latest_frame
        

class MultiSensorRecorder:
    """Main recorder managing ZED sensors only"""
    def __init__(self, ros_node, output_base_dir="./data"):
        self.output_base_dir = output_base_dir
        self.current_session_dir = None
        self.ros_node = ros_node
        self.zed_recorders = {}

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
        for recorder in self.zed_recorders.values():
            recorder.start_recording(self.current_session_dir)
        print("\n[Recorder] All ZED sensors recording started\n")

    def stop_recording(self):
        for recorder in self.zed_recorders.values():
            recorder.stop_recording()
        print("\n[Recorder] All ZED sensors recording stopped\n")

    def shutdown(self):
        for recorder in self.zed_recorders.values():
            recorder.stop()