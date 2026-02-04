import os
import time
import threading
import signal
import sys
import pyzed.sl as sl
import cv2
import numpy as np
import base64

# ROS2 imports
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

stop_event = threading.Event()

class ZEDCameraRecorder(threading.Thread):
    def __init__(self, serial_number, name, ros_node, socketio=None, interval=1.0, always_stream=False):
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
        
        # Prepare image containers
        self.image_rgb = sl.Mat()
        self.image_depth = sl.Mat()
        self.runtime = sl.RuntimeParameters()
        
        # ROS2 Publishers (only created when needed)
        self.rgb_pub = None
        self.depth_pub = None
        
        print(f"[{self.name}] Camera initialized - S/N: {serial_number}")
        if self.always_stream:
            print(f"[{self.name}] Always streaming to web")

    def start_recording(self, output_dir):
        """Start saving frames to disk and publishing to ROS2"""
        self.current_output_dir = output_dir
        
        # Create output directories
        self.rgb_dir = os.path.join(output_dir, self.name, "rgb")
        self.depth_dir = os.path.join(output_dir, self.name, "depth")
        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        
        # Create ROS2 publishers if not exists
        if self.rgb_pub is None:
            self.rgb_pub = self.ros_node.create_publisher(Image, f'/{self.name}/rgb', 10)
            self.depth_pub = self.ros_node.create_publisher(Image, f'/{self.name}/depth', 10)
            print(f"[{self.name}] ROS2 publishers created")
        
        self.recording = True
        print(f"[{self.name}] Recording started to: {output_dir}")
    
    def stop_recording(self):
        """Stop saving frames to disk and publishing to ROS2"""
        self.recording = False
        self.current_output_dir = None
        print(f"[{self.name}] Recording stopped")
        
    def get_latest_frame(self):
        """Get latest frame"""
        with self.frame_lock:
            return self.latest_frame
        
        
    def run(self):
        frame_count = 0
        last_process_time = time.time()
        
        print(f"[{self.name}] Thread started")
        
        while self.running and not stop_event.is_set():
            if self.cam.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
                current_time = time.time()
                
                # Check if it's time to process frame
                if current_time - last_process_time >= self.interval:
                    # Retrieve RGB image
                    self.cam.retrieve_image(self.image_rgb, sl.VIEW.LEFT)
                    rgb_np = self.image_rgb.get_data()
                    
                    # Retrieve Depth image
                    self.cam.retrieve_measure(self.image_depth, sl.MEASURE.DEPTH)
                    depth_np = self.image_depth.get_data()
                    
                    timestamp = int(current_time * 1000)
                    
                    if self.always_stream and self.socketio:
                        try:
                            rgb_np = cv2.cvtColor(rgb_np, cv2.COLOR_RGBA2RGB)
                            
                            H, W, _ = rgb_np.shape
                            scale = 0.2
                            rgb_np = cv2.resize(rgb_np, (int(round(W*scale)), int(round(H*scale)))) 
                            _, buffer = cv2.imencode('.jpg', rgb_np, [cv2.IMWRITE_JPEG_QUALITY, 70])
                            
                            with self.frame_lock:
                                self.latest_frame = buffer.tobytes()
                                
                        except Exception as e:
                            print(f"[{self.name}] Web streaming error: {e}")
                            
                    # Save to disk and publish to ROS2 if recording
                    if self.recording and self.current_output_dir:
                        # Save files
                        rgb_path = os.path.join(self.rgb_dir, f"{timestamp:016d}.png")
                        depth_path = os.path.join(self.depth_dir, f"{timestamp:016d}.npy")
                        
                        cv2.imwrite(rgb_path, cv2.cvtColor(rgb_np, cv2.COLOR_RGBA2RGB))
                        np.save(depth_path, depth_np)
                        
                        frame_count += 1
                        if frame_count % 10 == 0:
                            print(f"[{self.name}] Saved frame {frame_count}")
                        
                        # Publish to ROS2
                        try:
                            # RGB
                            rgb_msg = self.bridge.cv2_to_imgmsg(
                                cv2.cvtColor(rgb_np, cv2.COLOR_RGBA2RGB), 
                                encoding="rgb8"
                            )
                            rgb_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
                            rgb_msg.header.frame_id = f'zed_{self.name}_camera'
                            self.rgb_pub.publish(rgb_msg)
                            
                            # Depth
                            depth_msg = self.bridge.cv2_to_imgmsg(depth_np, encoding="32FC1")
                            depth_msg.header.stamp = rgb_msg.header.stamp
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