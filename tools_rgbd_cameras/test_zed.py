import os
import time
import threading
import argparse
import signal
import sys
import pyzed.sl as sl
import cv2
import numpy as np

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

stop_event = threading.Event()

class ZEDCameraRecorder(threading.Thread):
    def __init__(self, serial_number, name, base_path, interval, ros_node):
        super().__init__(name=name)
        self.serial = serial_number
        self.name = name
        self.base_path = base_path
        self.interval = interval
        self.ros_node = ros_node
        self.running = True
        self.bridge = CvBridge()
        
        # Create output directories
        self.rgb_dir = os.path.join(base_path, name, "rgb")
        self.depth_dir = os.path.join(base_path, name, "depth")
        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        
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
        
        # ROS2 Publishers
        self.rgb_pub = self.ros_node.create_publisher(
            Image, f'rgb_{name}', 10)
        self.depth_pub = self.ros_node.create_publisher(
            Image, f'depth_{name}', 10)
        
        print(f"[{self.name}] Camera initialized - S/N: {serial_number}")

    def run(self):
        frame_count = 0
        last_save_time = time.time()
        
        print(f"[{self.name}] Recording started")
        
        while self.running and not stop_event.is_set():
            if self.cam.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
                current_time = time.time()
                
                # Check if it's time to save/publish
                if current_time - last_save_time >= self.interval:
                    # Retrieve RGB image
                    self.cam.retrieve_image(self.image_rgb, sl.VIEW.LEFT)
                    rgb_np = self.image_rgb.get_data()
                    
                    # Retrieve Depth image
                    self.cam.retrieve_measure(self.image_depth, sl.MEASURE.DEPTH)
                    depth_np = self.image_depth.get_data()
                    
                    # Save to disk
                    timestamp = int(current_time * 1000)  # milliseconds
                    rgb_path = os.path.join(self.rgb_dir, f"{timestamp:016d}.png")
                    depth_path = os.path.join(self.depth_dir, f"{timestamp:016d}.npy")
                    
                    # Save RGB as PNG
                    cv2.imwrite(rgb_path, cv2.cvtColor(rgb_np, cv2.COLOR_RGBA2BGR))
                    
                    # Save Depth as numpy array (preserves float values)
                    np.save(depth_path, depth_np)
                    
                    # Publish to ROS2
                    try:
                        # RGB (convert RGBA to RGB)
                        rgb_msg = self.bridge.cv2_to_imgmsg(
                            cv2.cvtColor(rgb_np, cv2.COLOR_RGBA2RGB), 
                            encoding="rgb8"
                        )
                        rgb_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
                        rgb_msg.header.frame_id = f'zed_{self.name}_camera'
                        self.rgb_pub.publish(rgb_msg)
                        
                        # Depth (convert to 32FC1)
                        depth_msg = self.bridge.cv2_to_imgmsg(
                            depth_np, 
                            encoding="32FC1"
                        )
                        depth_msg.header.stamp = rgb_msg.header.stamp
                        depth_msg.header.frame_id = f'zed_{self.name}_camera'
                        self.depth_pub.publish(depth_msg)
                        
                    except Exception as e:
                        print(f"[{self.name}] ROS2 publish error: {e}")
                    
                    frame_count += 1
                    if frame_count % 10 == 0:
                        print(f"[{self.name}] Saved frame {frame_count}")
                    
                    last_save_time = current_time
            else:
                time.sleep(0.001)
        
        print(f"[{self.name}] Exiting safely. Total frames: {frame_count}")

    def stop(self):
        self.running = False
        stop_event.set()
        self.join()
        self.cam.close()
        print(f"[{self.name}] Stopped.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="./output/", help="Output directory path")
    parser.add_argument("--interval", type=float, default=1.0, 
                       help="Save/publish interval in seconds (default: 1.0)")
    args = parser.parse_args()

    base_path = os.path.expanduser(args.output)
    os.makedirs(base_path, exist_ok=True)

    # Initialize ROS2
    rclpy.init()
    node = rclpy.create_node('zed_dual_camera_recorder')

    print("ZED Dual Camera Recorder")
    print(f"Output: {base_path}")
    print(f"Interval: {args.interval}s")
    
    # Create camera recorders
    rec_front = ZEDCameraRecorder(
        serial_number=48335070, 
        name="front", 
        base_path=base_path, 
        interval=args.interval,
        ros_node=node
    )
    rec_back = ZEDCameraRecorder(
        serial_number=49537850, 
        name="back", 
        base_path=base_path, 
        interval=args.interval,
        ros_node=node
    )

    # Start recording
    rec_front.start()
    rec_back.start()

    def signal_handler(sig, frame):
        print("\n[🛑] Stopping...")
        rec_front.stop()
        rec_back.stop()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        signal_handler(None, None)


if __name__ == "__main__":
    main()