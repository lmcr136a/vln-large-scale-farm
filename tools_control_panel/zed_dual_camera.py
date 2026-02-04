import os
import time
import threading
import signal
import sys
import pyzed.sl as sl
import cv2
import numpy as np
import pandas as pd
from collections import defaultdict

# ROS2 imports
import rclpy
from sensor_msgs.msg import Image, PointCloud2, Imu
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge

stop_event = threading.Event()

class ZEDCameraRecorder(threading.Thread):
    def __init__(self, serial_number, name, ros_node, socketio=None, interval=0.1, 
                 always_stream=False, pairing_threshold_ms=50):
        super().__init__(name=name)
        self.serial = serial_number
        self.name = name
        self.interval = interval
        self.ros_node = ros_node
        self.socketio = socketio
        self.running = True
        self.bridge = CvBridge()
        self.always_stream = always_stream
        self.recording = False
        self.current_output_dir = None
        self.latest_frame = None 
        self.frame_lock = threading.Lock()
        self.pairing_threshold_ms = pairing_threshold_ms
        
        # Timestamp tracking for pairing
        self.zed_timestamps = []  # [(sec, nanosec, rgb_path, depth_path), ...]
        self.lidar_timestamps = []  # [(sec, nanosec, lidar_path), ...]
        self.timestamps_lock = threading.Lock()
        
        # IMU data storage
        self.imu_data = []  # [(sec, nanosec, ax, ay, az, gx, gy, gz, qx, qy, qz, qw), ...]
        self.imu_lock = threading.Lock()
        
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
        
        # ROS2 Publishers and Subscribers
        self.rgb_pub = None
        self.depth_pub = None
        self.lidar_sub = None
        self.imu_sub = None
        
        print(f"[{self.name}] Camera initialized - S/N: {serial_number}")
        if self.always_stream:
            print(f"[{self.name}] Always streaming to web")

    def start_recording(self, output_dir):
        """Start recording with proper directory structure"""
        self.current_output_dir = output_dir
        
        # Create ECCV-style directory structure
        self.zed_dir = os.path.join(output_dir, "zed", self.name)
        self.rgb_dir = os.path.join(self.zed_dir, "rgb")
        self.depth_dir = os.path.join(self.zed_dir, "depth")
        self.lidar_dir = os.path.join(output_dir, "livox", "pointcloud")
        self.imu_dir = os.path.join(output_dir, "imu")
        
        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        os.makedirs(self.lidar_dir, exist_ok=True)
        os.makedirs(self.imu_dir, exist_ok=True)
        
        # Create ROS2 publishers
        if self.rgb_pub is None:
            self.rgb_pub = self.ros_node.create_publisher(Image, f'/{self.name}/rgb', 10)
            self.depth_pub = self.ros_node.create_publisher(Image, f'/{self.name}/depth', 10)
            print(f"[{self.name}] ROS2 publishers created")
        
        # Create LiDAR subscriber
        if self.lidar_sub is None:
            self.lidar_sub = self.ros_node.create_subscription(
                PointCloud2,
                '/lidar3d',
                self.lidar_callback,
                10
            )
            print(f"[{self.name}] LiDAR subscriber created")
        
        # Create IMU subscriber (100Hz expected)
        if self.imu_sub is None:
            self.imu_sub = self.ros_node.create_subscription(
                Imu,
                '/imu',
                self.imu_callback,
                100
            )
            print(f"[{self.name}] IMU subscriber created")
        
        # Reset timestamp tracking
        with self.timestamps_lock:
            self.zed_timestamps = []
            self.lidar_timestamps = []
        
        with self.imu_lock:
            self.imu_data = []
        
        self.recording = True
        print(f"[{self.name}] Recording started to: {output_dir}")
        print(f"[{self.name}] Pairing threshold: {self.pairing_threshold_ms}ms")
        print(f"[{self.name}] ZED/LiDAR: 10Hz, IMU: 100Hz")
    
    def stop_recording(self):
        """Stop recording and generate metadata"""
        self.recording = False
        
        if self.current_output_dir:
            print(f"[{self.name}] Generating metadata...")
            self._save_imu_data()
            self._generate_pairs_csv()
            self._save_pairing_statistics()
        
        self.current_output_dir = None
        print(f"[{self.name}] Recording stopped")
    
    def imu_callback(self, msg):
        """Save IMU data at 100Hz with original timestamp"""
        if not self.recording or not self.current_output_dir:
            return
        
        try:
            sec = msg.header.stamp.sec
            nanosec = msg.header.stamp.nanosec
            
            # Linear acceleration (m/s²)
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            
            # Angular velocity (rad/s)
            gx = msg.angular_velocity.x
            gy = msg.angular_velocity.y
            gz = msg.angular_velocity.z
            
            # Orientation (quaternion)
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w
            
            with self.imu_lock:
                self.imu_data.append((sec, nanosec, ax, ay, az, gx, gy, gz, qx, qy, qz, qw))
                
        except Exception as e:
            print(f"[{self.name}] IMU save error: {e}")
    
    def _save_imu_data(self):
        """Save all IMU data to CSV"""
        with self.imu_lock:
            if len(self.imu_data) == 0:
                print(f"[{self.name}] No IMU data to save")
                return
            
            df = pd.DataFrame(self.imu_data, columns=[
                'timestamp_sec', 'timestamp_nsec',
                'accel_x_mps2', 'accel_y_mps2', 'accel_z_mps2',
                'gyro_x_radps', 'gyro_y_radps', 'gyro_z_radps',
                'quat_x', 'quat_y', 'quat_z', 'quat_w'
            ])
            
            imu_path = os.path.join(self.imu_dir, 'imu_data.csv')
            df.to_csv(imu_path, index=False)
            print(f"[{self.name}] Saved {len(self.imu_data)} IMU samples to {imu_path}")
    
    def lidar_callback(self, msg):
        """Save LiDAR point cloud with original timestamp"""
        if not self.recording or not self.current_output_dir:
            return
        
        try:
            sec = msg.header.stamp.sec
            nanosec = msg.header.stamp.nanosec
            timestamp_str = f"{sec}_{nanosec:09d}"
            
            # Convert PointCloud2 to numpy array (x, y, z, intensity)
            points_list = []
            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                points_list.append(point)
            
            if len(points_list) > 0:
                points_np = np.array(points_list, dtype=np.float32)
                
                # Save as binary file (KITTI format)
                lidar_path = os.path.join(self.lidar_dir, f"{timestamp_str}.bin")
                points_np.tofile(lidar_path)
                
                # Track timestamp for pairing
                with self.timestamps_lock:
                    self.lidar_timestamps.append((sec, nanosec, lidar_path))
                
        except Exception as e:
            print(f"[{self.name}] LiDAR save error: {e}")
    
    def _generate_pairs_csv(self):
        """Generate pairs.csv with pairing information"""
        with self.timestamps_lock:
            zed_ts = np.array([(s + n * 1e-9, i) for i, (s, n, _, _) in enumerate(self.zed_timestamps)])
            lidar_ts = np.array([(s + n * 1e-9, i) for i, (s, n, _) in enumerate(self.lidar_timestamps)])
        
        if len(zed_ts) == 0 or len(lidar_ts) == 0:
            print(f"[{self.name}] No data to pair")
            return
        
        pairs = []
        threshold_sec = self.pairing_threshold_ms / 1000.0
        
        # Pairing strategy: LiDAR timestamp as reference
        for lidar_t, lidar_idx in lidar_ts:
            # Find nearest ZED timestamp
            dt_array = np.abs(zed_ts[:, 0] - lidar_t)
            nearest_idx = np.argmin(dt_array)
            dt = dt_array[nearest_idx]
            
            # Only pair if within threshold
            if dt <= threshold_sec:
                zed_idx = int(zed_ts[nearest_idx, 1])
                lidar_idx = int(lidar_idx)
                
                zed_sec, zed_nsec, rgb_path, depth_path = self.zed_timestamps[zed_idx]
                lidar_sec, lidar_nsec, lidar_path = self.lidar_timestamps[lidar_idx]
                
                pairs.append({
                    'pair_id': len(pairs),
                    'zed_sec': zed_sec,
                    'zed_nsec': zed_nsec,
                    'lidar_sec': lidar_sec,
                    'lidar_nsec': lidar_nsec,
                    'dt_ms': dt * 1000,
                    'rgb_path': os.path.relpath(rgb_path, self.current_output_dir),
                    'depth_path': os.path.relpath(depth_path, self.current_output_dir),
                    'lidar_path': os.path.relpath(lidar_path, self.current_output_dir)
                })
        
        # Save pairs.csv
        if len(pairs) > 0:
            df = pd.DataFrame(pairs)
            csv_path = os.path.join(self.current_output_dir, f'pairs_{self.name}.csv')
            df.to_csv(csv_path, index=False)
            print(f"[{self.name}] Saved {len(pairs)} pairs to {csv_path}")
            print(f"[{self.name}] Unpaired: ZED={len(self.zed_timestamps)-len(pairs)}, LiDAR={len(self.lidar_timestamps)-len(pairs)}")
        else:
            print(f"[{self.name}] No valid pairs found within {self.pairing_threshold_ms}ms threshold")
    
    def _save_pairing_statistics(self):
        """Save pairing statistics as CSV for paper"""
        csv_path = os.path.join(self.current_output_dir, f'pairs_{self.name}.csv')
        
        if not os.path.exists(csv_path):
            return
        
        df = pd.DataFrame(pairs)
        dt_ms = df['dt_ms'].values
        
        stats = {
            'metric': ['mean_ms', 'median_ms', 'std_ms', 'p95_ms', 'max_ms', 'min_ms',
                      'total_pairs', 'total_zed_frames', 'total_lidar_scans', 'unpaired_zed',
                      'unpaired_lidar', 'pairing_rate', 'threshold_ms'],
            'value': [
                np.mean(dt_ms),
                np.median(dt_ms),
                np.std(dt_ms),
                np.percentile(dt_ms, 95),
                np.max(dt_ms),
                np.min(dt_ms),
                len(df),
                len(self.zed_timestamps),
                len(self.lidar_timestamps),
                len(self.zed_timestamps) - len(df),
                len(self.lidar_timestamps) - len(df),
                len(df) / max(len(self.zed_timestamps), len(self.lidar_timestamps)),
                self.pairing_threshold_ms
            ]
        }
        
        stats_df = pd.DataFrame(stats)
        stats_path = os.path.join(self.current_output_dir, f'pairing_stats_{self.name}.csv')
        stats_df.to_csv(stats_path, index=False)
        
        print(f"[{self.name}] Pairing stats saved: Δt_mean={np.mean(dt_ms):.1f}ms, Δt_p95={np.percentile(dt_ms, 95):.1f}ms")
        
    def get_latest_frame(self):
        """Get latest frame for web streaming"""
        with self.frame_lock:
            return self.latest_frame
        
    def run(self):
        frame_count = 0
        last_process_time = time.time()
        
        print(f"[{self.name}] Thread started")
        
        while self.running and not stop_event.is_set():
            if self.cam.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
                current_time = time.time()
                
                if current_time - last_process_time >= self.interval:
                    # Get ZED hardware timestamp
                    zed_timestamp = self.cam.get_timestamp(sl.TIME_REFERENCE.IMAGE)
                    sec = int(zed_timestamp.get_seconds())
                    nanosec = int(zed_timestamp.get_nanoseconds())
                    timestamp_str = f"{sec}_{nanosec:09d}"
                    
                    # Retrieve images
                    self.cam.retrieve_image(self.image_rgb, sl.VIEW.LEFT)
                    rgb_np = self.image_rgb.get_data()
                    
                    self.cam.retrieve_measure(self.image_depth, sl.MEASURE.DEPTH)
                    depth_np = self.image_depth.get_data()
                    
                    # Web streaming
                    if self.always_stream and self.socketio:
                        try:
                            rgb_display = cv2.cvtColor(rgb_np, cv2.COLOR_RGBA2RGB)
                            H, W, _ = rgb_display.shape
                            scale = 0.2
                            rgb_display = cv2.resize(rgb_display, (int(round(W*scale)), int(round(H*scale)))) 
                            _, buffer = cv2.imencode('.jpg', rgb_display, [cv2.IMWRITE_JPEG_QUALITY, 70])
                            
                            with self.frame_lock:
                                self.latest_frame = buffer.tobytes()
                                
                        except Exception as e:
                            print(f"[{self.name}] Web streaming error: {e}")
                    
                    # Save to disk with original timestamp (10Hz)
                    if self.recording and self.current_output_dir:
                        rgb_path = os.path.join(self.rgb_dir, f"{timestamp_str}.png")
                        depth_path = os.path.join(self.depth_dir, f"{timestamp_str}.npy")
                        
                        cv2.imwrite(rgb_path, cv2.cvtColor(rgb_np, cv2.COLOR_RGBA2RGB))
                        np.save(depth_path, depth_np)
                        
                        # Track timestamp for pairing
                        with self.timestamps_lock:
                            self.zed_timestamps.append((sec, nanosec, rgb_path, depth_path))
                        
                        frame_count += 1
                        if frame_count % 30 == 0:
                            print(f"[{self.name}] Saved {frame_count} frames")
                        
                        # Publish to ROS2
                        try:
                            ros_stamp = self.ros_node.get_clock().now().to_msg()
                            ros_stamp.sec = sec
                            ros_stamp.nanosec = nanosec
                            
                            rgb_msg = self.bridge.cv2_to_imgmsg(
                                cv2.cvtColor(rgb_np, cv2.COLOR_RGBA2RGB), 
                                encoding="rgb8"
                            )
                            rgb_msg.header.stamp = ros_stamp
                            rgb_msg.header.frame_id = f'zed_{self.name}_camera'
                            self.rgb_pub.publish(rgb_msg)
                            
                            depth_msg = self.bridge.cv2_to_imgmsg(depth_np, encoding="32FC1")
                            depth_msg.header.stamp = ros_stamp
                            depth_msg.header.frame_id = f'zed_{self.name}_camera'
                            self.depth_pub.publish(depth_msg)
                            
                        except Exception as e:
                            print(f"[{self.name}] ROS2 publish error: {e}")
                    
                    last_process_time = current_time
            else:
                time.sleep(0.001)
        
        print(f"[{self.name}] Exiting safely. Total frames saved: {frame_count}")

    def stop(self):
        self.running = False
        stop_event.set()
        self.join()
        self.cam.close()
        print(f"[{self.name}] Stopped.")