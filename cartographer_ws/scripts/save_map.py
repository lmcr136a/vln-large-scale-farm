#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
import numpy as np
from PIL import Image, ImageDraw
import time
import math
import yaml
from pathlib import Path
from datetime import datetime
from tf2_ros import Buffer, TransformListener
from rclpy.parameter import Parameter


RED1 = (200, 60, 40)
BLUE1 = (20, 30, 140)
BLUE2 = (55, 150, 185)
YELLOW = (255, 255, 0)


class MapSaver(Node):
    def __init__(self, use_sim_time: bool = False):
        super().__init__('map_saver')

        # set use_sim_time (already declared by ROS2)
        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)
        ])
        self.is_simulation = use_sim_time
        self.workspace_dir = Path(__file__).resolve().parent.parent
        self.output_dir = self.workspace_dir / 'output'
        
        # Create timestamped figures directory
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.figures_dir = self.output_dir / 'figures' / timestamp
        
        self.output_dir.mkdir(exist_ok=True)
        self.figures_dir.mkdir(parents=True, exist_ok=True)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Map subscription
        self.map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # Trajectory: use MarkerArray if available (sim), else self-track (real)
        self.trajectory_subscription = self.create_subscription(
            MarkerArray, '/trajectory_node_list', self.trajectory_callback, 10)
        
        self.last_save_time = 0
        self.save_interval = 1
        self.trajectory_nodes = []  # For simulation (MarkerArray)
        self.trajectory = []  # For real robot (self-tracked)
        self.save_count = 0
        
        self.get_logger().info(f'Map Saver started (simulation={self.is_simulation})')
        self.get_logger().info(f'Figures directory: {self.figures_dir}')
        
    def trajectory_callback(self, msg):
        """Called only in simulation when /trajectory_node_list exists"""
        self.trajectory_nodes = []
        for marker in msg.markers:
            for point in marker.points:
                self.trajectory_nodes.append((point.x, point.y))
        
    def get_robot_pose_from_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except:
            return None
            
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        
        # Yaw calculation (positive for simulation, negative for real)
        if self.is_simulation:
            yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        else:
            yaw = -math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        
        return (x, y, yaw)
        
    def map_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_save_time < self.save_interval:
            return  
        
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        width = msg.info.width
        height = msg.info.height
        
        robot_pose = self.get_robot_pose_from_tf()
        
        # Track trajectory for real robot (if no MarkerArray)
        if not self.is_simulation and robot_pose:
            x, y, _ = robot_pose
            self.trajectory.append((x, y))
            if len(self.trajectory) > 200:
                self.trajectory.pop(0)
        
        # Convert occupancy grid to image
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        img = np.zeros((height, width), dtype=np.uint8)
        img[data == -1] = 127  # Unknown
        img[data == 0] = 255   # Free
        img[data > 0] = 0      # Occupied
        
        img = np.flipud(img)
        im = Image.fromarray(img).convert('RGB')
        draw = ImageDraw.Draw(im)
        
        # Draw grid
        grid_spacing = int(5.0 / resolution)
        for i in range(0, width, grid_spacing):
            draw.line([(i, 0), (i, height-1)], fill=(100, 100, 100), width=1)
        for j in range(0, height, grid_spacing):
            draw.line([(0, j), (width-1, j)], fill=(100, 100, 100), width=1)
        
        # Draw origin
        origin_img_x = int((0 - origin_x) / resolution)
        origin_img_y = height - 1 - int((0 - origin_y) / resolution)
        if 0 <= origin_img_x < width and 0 <= origin_img_y < height:
            cross_size = 20
            draw.line([(origin_img_x - cross_size, origin_img_y), 
                      (origin_img_x + cross_size, origin_img_y)], 
                     fill=(255, 0, 0), width=3)
            draw.line([(origin_img_x, origin_img_y - cross_size), 
                      (origin_img_x, origin_img_y + cross_size)], 
                     fill=(255, 0, 0), width=3)
        
        # Draw trajectory (use MarkerArray if available, else self-tracked)
        trajectory_to_draw = self.trajectory_nodes if self.trajectory_nodes else self.trajectory
        
        if self.is_simulation:
            # Simulation: draw nodes as dots
            for x, y in trajectory_to_draw:
                img_x = int((x - origin_x) / resolution)
                img_y = height - 1 - int((y - origin_y) / resolution)
                if 0 <= img_x < width and 0 <= img_y < height:
                    draw.ellipse([img_x-1, img_y-1, img_x+1, img_y+1], fill=BLUE2)
        else:
            # Real: draw trajectory as line
            for i in range(len(trajectory_to_draw) - 1):
                x1, y1 = trajectory_to_draw[i]
                x2, y2 = trajectory_to_draw[i + 1]
                img_x1 = int((x1 - origin_x) / resolution)
                img_y1 = height - 1 - int((y1 - origin_y) / resolution)
                img_x2 = int((x2 - origin_x) / resolution)
                img_y2 = height - 1 - int((y2 - origin_y) / resolution)
                if (0 <= img_x1 < width and 0 <= img_y1 < height and 
                    0 <= img_x2 < width and 0 <= img_y2 < height):
                    draw.line([(img_x1, img_y1), (img_x2, img_y2)], fill=(0, 255, 0), width=3)
        
        # Draw robot
        if robot_pose:
            x, y, yaw = robot_pose
            img_x = int((x - origin_x) / resolution)
            img_y = height - 1 - int((y - origin_y) / resolution)
            
            if 0 <= img_x < width and 0 <= img_y < height:
                robot_size = 10
                arrow_length = robot_size * 2.1
                
                draw.ellipse([img_x-robot_size, img_y-robot_size, 
                             img_x+robot_size, img_y+robot_size], 
                            fill=BLUE1, outline=RED1, width=2)
                
                end_x = img_x + arrow_length * math.cos(yaw)
                end_y = img_y - arrow_length * math.sin(yaw)
                draw.line([(img_x, img_y), (end_x, end_y)], fill=RED1, width=2)
                
                # Arrow head
                arrow_angle = 30 * math.pi / 180
                head_length = robot_size * 0.75
                left_x = end_x - head_length * math.cos(yaw - arrow_angle)
                left_y = end_y + head_length * math.sin(yaw - arrow_angle)
                right_x = end_x - head_length * math.cos(yaw + arrow_angle)
                right_y = end_y + head_length * math.sin(yaw + arrow_angle)
                draw.polygon([(end_x, end_y), (left_x, left_y), (right_x, right_y)], fill=RED1)
                
                # Info text
                info_text = f"Robot: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}deg)"
                bbox = draw.textbbox((0, 0), info_text)
                text_width = bbox[2] - bbox[0]
                text_height = bbox[3] - bbox[1]
                draw.rectangle([(10, 10), (10 + text_width + 10, 10 + text_height + 10)], fill=(0, 0, 0))
                draw.text((15, 15), info_text, fill=YELLOW)
        
        
        if im.size[0] > 0 and im.size[1] > 0:
            im.save(str(self.output_dir / 'map_latest.png'), optimize=False, quality=95)
        else:
            print(f"Warning: Invalid map size {im.size}, skipping save")
        
        # Save lightweight version for figures (compressed)
        im_small = im.resize((int(round(width // 2)), int(round(height // 2))))
        try:
            im_small.save(
                str(self.figures_dir / f'map_{self.save_count:04d}.png'),
                optimize=True,
                quality=60
            )
            self.save_count += 1
        except:
            print("Map save failed")
            
                
        if robot_pose:
            x, y, yaw = robot_pose
        else:
            x, y, yaw = 0.0, 0.0, 0.0
        # Save metadata
        map_yaml = {
            'image': 'map_latest.png',
            'resolution': float(resolution),
            'origin': [float(origin_x), float(origin_y), 0.0],
            'robot_x': float(x) if robot_pose else 0.0,
            'robot_y': float(y) if robot_pose else 0.0,
            'robot_yaw': float(yaw) if robot_pose else 0.0
        }
        
        with open(str(self.output_dir / 'map_latest.yaml'), 'w') as f:
            yaml.dump(map_yaml, f, default_flow_style=False)
        
        self.last_save_time = current_time

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', default='real', choices=['real', 'sim'])
    args = parser.parse_args()
    
    rclpy.init()
    
    use_sim_time = (args.mode == 'sim')
    node = MapSaver(use_sim_time=use_sim_time)
    
    node.get_logger().info(f'Map Saver started (mode={args.mode}, use_sim_time={use_sim_time})')
    
    rclpy.spin(node)

if __name__ == '__main__':
    main()