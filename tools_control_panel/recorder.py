import os
import time
import threading
import pyzed.sl as sl
import cv2
import numpy as np
from datetime import datetime

# ROS2 imports
import rclpy
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2

stop_event = threading.Event()


class ZEDSVORecorder(threading.Thread):
    """Records ZED camera to SVO2 file with samples every N seconds"""
    def __init__(self, serial_number, name, output_dir, ros_node, socketio=None, 
                 sample_interval_sec=300, always_stream=False):
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
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # SVO recording path (will be set when recording starts)
        self.svo_path = None
        self.samples_dir = None
        
        # Initialize camera
        self.cam = sl.Camera()
        init = sl.InitParameters()
        input_type = sl.InputType()
        input_type.set_from_serial_number(serial_number)
        init.input = input_type
        init.depth_mode = sl.DEPTH_MODE.NEURAL
        init.coordinate_units = sl.UNIT.METER
        init.camera_resolution = sl.RESOLUTION.HD1080
        init.camera_fps = 30
        
        status = self.cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED camera {serial_number}: {status}")
        
        self.image_rgb = sl.Mat()
        self.image_depth = sl.Mat()
        self.runtime = sl.RuntimeParameters()
        
        print(f"[{self.name}] Camera initialized - S/N: {serial_number}")
        if self.always_stream:
            print(f"[{self.name}] Always streaming to web")
    
    def start_recording(self, output_dir):
        """Start SVO2 recording"""
        self.output_dir = output_dir
        self.svo_path = os.path.join(output_dir, f"rgbd_{self.name}.svo2")
        self.samples_dir = os.path.join(output_dir, "samples")
        os.makedirs(self.samples_dir, exist_ok=True)
        
        # SVO recording parameters
        recording_params = sl.RecordingParameters()
        recording_params.compression_mode = sl.SVO_COMPRESSION_MODE.H265  # Best compression
        recording_params.video_filename = self.svo_path
        recording_params.transcode_streaming_input = True
        
        err = self.cam.enable_recording(recording_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to enable SVO recording: {err}")
        
        self.recording = True
        self.last_sample_time = time.time() - self.sample_interval_sec - 1
        print(f"[{self.name}] SVO2 recording started: {self.svo_path}")
        print(f"[{self.name}] Samples will be saved every {self.sample_interval_sec}s")
    
    def stop_recording(self):
        """Stop SVO2 recording"""
        self.recording = False
        self.cam.disable_recording()
        print(f"[{self.name}] SVO2 recording stopped: {self.svo_path}")
    
    def save_sample(self, timestamp_str):
        """Save sample RGB and Depth images"""
        self.cam.retrieve_image(self.image_rgb, sl.VIEW.LEFT)
        rgb_np = self.image_rgb.get_data()
        
        self.cam.retrieve_measure(self.image_depth, sl.MEASURE.DEPTH)
        depth_np = self.image_depth.get_data()
        
        # Convert and save RGB
        rgb_img = cv2.cvtColor(rgb_np, cv2.COLOR_RGBA2RGB)
        rgb_path = os.path.join(self.samples_dir, f"{timestamp_str}_{self.name}.jpg")
        cv2.imwrite(rgb_path, rgb_img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        
        # Convert depth to 16-bit PNG (mm units, lossless compression)
        depth_mm = np.clip(depth_np, 0, 65535).astype(np.uint16)
        depth_path = os.path.join(self.samples_dir, f"{timestamp_str}_{self.name}_depth.png")
        cv2.imwrite(depth_path, depth_mm)
        
        print(f"[{self.name}] Sample saved: {timestamp_str}")
    
    def get_latest_frame(self):
        """Get latest frame for web streaming"""
        with self.frame_lock:
            return self.latest_frame
    
    def run(self):
        frame_count = 0
        print(f"[{self.name}] Thread started")
        
        while self.running and not stop_event.is_set():
            if self.cam.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
                frame_count += 1
                
                # Web streaming
                if self.always_stream and self.socketio:
                    try:
                        self.cam.retrieve_image(self.image_rgb, sl.VIEW.LEFT)
                        rgb_np = self.image_rgb.get_data()
                        rgb_display = cv2.cvtColor(rgb_np.copy(), cv2.COLOR_RGBA2RGB)
                        H, W, _ = rgb_display.shape
                        scale = 0.2
                        rgb_display = cv2.resize(rgb_display, (int(W*scale), int(H*scale)))
                        _, buffer = cv2.imencode('.jpg', rgb_display, [cv2.IMWRITE_JPEG_QUALITY, 70])
                        
                        with self.frame_lock:
                            self.latest_frame = buffer.tobytes()
                    except Exception as e:
                        print(f"[{self.name}] Web streaming error: {e}")
                
                # Save sample every N seconds
                if self.recording:
                    current_time = time.time()
                    if current_time - self.last_sample_time >= self.sample_interval_sec:
                        timestamp_str = datetime.now().strftime("%m%d_%H%M")
                        self.save_sample(timestamp_str)
                        self.last_sample_time = current_time
                    
                    if frame_count % 300 == 0:  # Every 10 seconds at 30fps
                        print(f"[{self.name}] Recorded {frame_count} frames")
            else:
                time.sleep(0.001)
        
        print(f"[{self.name}] Exiting safely. Total frames: {frame_count}")
    
    def stop(self):
        self.running = False
        stop_event.set()
        self.join()
        self.cam.close()
        print(f"[{self.name}] Stopped.")


class LiDARRecorder:
    """Records LiDAR point clouds and IMU to binary files"""
    def __init__(self, output_dir, ros_node):
        self.output_dir = output_dir
        self.ros_node = ros_node
        self.recording = False
        
        # Binary file handles
        self.pc_file = None
        self.imu_file = None
        
        # ROS subscribers
        self.lidar_sub = None
        self.imu_sub = None
        
        print("[LiDAR] Recorder initialized")
    
    def start_recording(self):
        """Start recording LiDAR data"""
        pc_path = os.path.join(self.output_dir, "lidar_pointcloud.bin")
        imu_path = os.path.join(self.output_dir, "lidar_imu.bin")
        
        self.pc_file = open(pc_path, 'wb')
        self.imu_file = open(imu_path, 'wb')
        
        # Create ROS2 subscribers
        self.lidar_sub = self.ros_node.create_subscription(
            PointCloud2, '/lidar3d', self.lidar_callback, 10
        )
        self.imu_sub = self.ros_node.create_subscription(
            Imu, '/imu', self.imu_callback, 100
        )
        
        self.recording = True
        print(f"[LiDAR] Recording started")
        print(f"  - PointCloud: {pc_path}")
        print(f"  - IMU: {imu_path}")
    
    def stop_recording(self):
        """Stop recording"""
        self.recording = False
        time.sleep(0.1)
        
        if self.pc_file:
            self.pc_file.close()
        if self.imu_file:
            self.imu_file.close()
        
        print("[LiDAR] Recording stopped")
    
    def lidar_callback(self, msg):
        """Save LiDAR point cloud with timestamp"""
        if not self.recording:
            return
        
        try:
            sec = msg.header.stamp.sec
            nanosec = msg.header.stamp.nanosec
            
            # Extract points
            points_list = []
            for data in point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                points_list.append([data[0], data[1], data[2], data[3]])
            
            if len(points_list) > 0:
                points_np = np.array(points_list, dtype=np.float32)
                
                # Write format: [timestamp_sec][timestamp_nsec][num_points][points_data]
                self.pc_file.write(np.array([sec], dtype=np.int32).tobytes())
                self.pc_file.write(np.array([nanosec], dtype=np.int32).tobytes())
                self.pc_file.write(np.array([len(points_np)], dtype=np.int32).tobytes())
                self.pc_file.write(points_np.tobytes())
                
        except Exception as e:
            print(f"[LiDAR] Point cloud save error: {e}")
    
    def imu_callback(self, msg):
        """Save IMU data with timestamp"""
        if not self.recording:
            return
        
        try:
            sec = msg.header.stamp.sec
            nanosec = msg.header.stamp.nanosec
            
            timestamp = np.array([sec, nanosec], dtype=np.int32)
            self.imu_file.write(timestamp.tobytes())

            sensor_data = np.array([
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            ], dtype=np.float32)
            self.imu_file.write(sensor_data.tobytes())
            
        except Exception as e:
            print(f"[LiDAR] IMU save error: {e}")


class MultiSensorRecorder:
    """Main recorder managing all sensors"""
    def __init__(self, ros_node, output_base_dir="./data"):
        self.output_base_dir = output_base_dir
        self.current_session_dir = None
        self.ros_node = ros_node
        
        # Recorders
        self.zed_recorders = {}
        self.lidar_recorder = LiDARRecorder(None, self.ros_node)
        
    def create_session(self, session_name=None):
        """Create new recording session directory"""
        if session_name is None:
            session_name = datetime.now().strftime("%Y%m%d_%H%M")
        
        self.current_session_dir = os.path.join(self.output_base_dir, session_name)
        os.makedirs(self.current_session_dir, exist_ok=True)
        
        # Update LiDAR recorder output dir
        self.lidar_recorder.output_dir = self.current_session_dir
        
        print(f"\n[Recorder] Session created: {self.current_session_dir}\n")
        return self.current_session_dir
    
    def add_zed_camera(self, serial_number, name, **kwargs):
        """Add ZED camera recorder"""
        recorder = ZEDSVORecorder(
            serial_number, name, self.current_session_dir, self.ros_node, **kwargs
        )
        self.zed_recorders[name] = recorder
        recorder.start()
        return recorder
    
    def start_recording(self):
        """Start all recorders"""
        for recorder in self.zed_recorders.values():
            recorder.start_recording(self.current_session_dir)
        
        self.lidar_recorder.start_recording()
        
        print("\n[Recorder] All sensors recording started\n")
    
    def stop_recording(self):
        """Stop all recorders"""
        for recorder in self.zed_recorders.values():
            recorder.stop_recording()
        
        self.lidar_recorder.stop_recording()
        
        print("\n[Recorder] All sensors recording stopped\n")
    
    def shutdown(self):
        """Clean shutdown"""
        for recorder in self.zed_recorders.values():
            recorder.stop()