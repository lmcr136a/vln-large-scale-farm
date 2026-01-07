#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import time
import math
import yaml
import os
from pathlib import Path
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        
        # Get workspace directory dynamically
        self.workspace_dir = Path(__file__).resolve().parent.parent
        self.output_dir = self.workspace_dir / 'output'
        self.figures_dir = self.output_dir / 'figures'
        
        # Create output directories if they don't exist
        self.output_dir.mkdir(exist_ok=True)
        self.figures_dir.mkdir(exist_ok=True)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        self.last_save_time = 0
        self.save_interval = 1
        self.trajectory = []
        self.robot_pose = None
        self.prev_map_info = None
        
    def get_robot_pose_from_tf(self):
        transform = self.tf_buffer.lookup_transform(
            'map',
            'livox_frame',
            rclpy.time.Time())
        
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        
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
        
        current_map_info = (width, height, origin_x, origin_y, resolution)
        if self.prev_map_info != current_map_info:
            self.prev_map_info = current_map_info
        
        robot_pose = self.get_robot_pose_from_tf()
        
        if robot_pose:
            x, y, yaw = robot_pose
            self.trajectory.append((x, y))
            if len(self.trajectory) > 200:
                self.trajectory.pop(0)
        
        data = np.array(msg.data).reshape((height, width))
        
        img = np.zeros((height, width), dtype=np.uint8)
        img[data == -1] = 127
        img[data == 0] = 255
        img[data > 0] = 0
        
        im = Image.fromarray(img).convert('RGB')
        draw = ImageDraw.Draw(im)
        
        small_font = ImageFont.load_default()
        
        grid_spacing_world = 5.0
        grid_spacing_cells = int(grid_spacing_world / resolution)
        
        for i in range(0, width, grid_spacing_cells):
            draw.line([(i, 0), (i, height-1)], fill=(100, 100, 100), width=1)
            world_x = origin_x + i * resolution
            draw.text((i+2, 5), f'{world_x:.0f}m', fill=(100, 100, 100), font=small_font)
        
        for j in range(0, height, grid_spacing_cells):
            draw.line([(0, j), (width-1, j)], fill=(100, 100, 100), width=1)
            world_y = origin_y + (height - 1 - j) * resolution
            draw.text((5, j+2), f'{world_y:.0f}m', fill=(100, 100, 100), font=small_font)
        
        origin_grid_x = int(round((0 - origin_x) / resolution))
        origin_grid_y = int(round((0 - origin_y) / resolution))
        origin_img_x = origin_grid_x
        origin_img_y = origin_grid_y
        
        if 0 <= origin_img_x < width and 0 <= origin_img_y < height:
            cross_size = 20
            draw.line([(origin_img_x - cross_size, origin_img_y), 
                      (origin_img_x + cross_size, origin_img_y)], 
                     fill=(255, 0, 0), width=3)
            draw.line([(origin_img_x, origin_img_y - cross_size), 
                      (origin_img_x, origin_img_y + cross_size)], 
                     fill=(255, 0, 0), width=3)
            draw.text((origin_img_x + 5, origin_img_y - 15), 
                     'World (0,0)', fill=(255, 0, 0), font=small_font)
        
        if len(self.trajectory) > 1:
            for i in range(len(self.trajectory) - 1):
                x1, y1 = self.trajectory[i]
                x2, y2 = self.trajectory[i + 1]
                
                gx1 = int(round((x1 - origin_x) / resolution))
                gy1 = int(round((y1 - origin_y) / resolution))
                gx2 = int(round((x2 - origin_x) / resolution))
                gy2 = int(round((y2 - origin_y) / resolution))
                
                ix1 = gx1
                iy1 = gy1
                ix2 = gx2
                iy2 = gy2
                
                if (0 <= ix1 < width and 0 <= iy1 < height and 
                    0 <= ix2 < width and 0 <= iy2 < height):
                    draw.line([(ix1, iy1), (ix2, iy2)], fill=(0, 255, 0), width=3)
        
        if robot_pose:
            x, y, yaw = robot_pose
            
            grid_x_int = int(round((x - origin_x) / resolution))
            grid_y_int = int(round((y - origin_y) / resolution))
            
            img_x = grid_x_int
            img_y = grid_y_int
            
            print(f"Map origin: ({origin_x:.2f}, {origin_y:.2f}), size: {width}x{height}")
            print(f"Robot world: ({x:.2f}, {y:.2f})")
            print(f"Robot grid: ({grid_x_int}, {grid_y_int})")
            print(f"Robot img: ({img_x}, {img_y})")
            print(f"In bounds: {0 <= img_x < width and 0 <= img_y < height}")
            print("-" * 50)
            
            if 0 <= img_x < width and 0 <= img_y < height:
                robot_size = 20
                draw.ellipse([img_x-robot_size, img_y-robot_size, 
                             img_x+robot_size, img_y+robot_size], 
                            fill=(0, 0, 255), outline=(255, 255, 0), width=4)
                
                arrow_length = 35
                img_yaw = yaw
                end_x = img_x + arrow_length * math.cos(img_yaw)
                end_y = img_y - arrow_length * math.sin(img_yaw)
                
                draw.line([(img_x, img_y), (end_x, end_y)], 
                         fill=(255, 255, 0), width=4)
                
                arrow_angle = 25 * math.pi / 180
                head_length = 15
                
                left_x = end_x - head_length * math.cos(img_yaw - arrow_angle)
                left_y = end_y + head_length * math.sin(img_yaw - arrow_angle)
                right_x = end_x - head_length * math.cos(img_yaw + arrow_angle)
                right_y = end_y + head_length * math.sin(img_yaw + arrow_angle)
                
                draw.polygon([(end_x, end_y), (left_x, left_y), (right_x, right_y)], 
                           fill=(255, 255, 0))
                
                info_text = f"Robot: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)"
                bbox = draw.textbbox((0, 0), info_text, font=small_font)
                text_width = bbox[2] - bbox[0]
                text_height = bbox[3] - bbox[1]
                draw.rectangle([(10, 10), (10 + text_width + 10, 10 + text_height + 10)], 
                              fill=(0, 0, 0))
                draw.text((15, 15), info_text, fill=(255, 255, 0), font=small_font)
            else:
                print(f"WARNING: Robot out of bounds!")
        
        # Save with dynamic paths
        im.save(str(self.figures_dir / f'building_{len(self.trajectory)}.png'))
        im.save(str(self.output_dir / 'map_latest.png'))
        
        map_yaml = {
            'image': 'map_latest.png',
            'resolution': float(resolution),
            'origin': [float(origin_x), float(origin_y), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
            'robot_x': float(x) if robot_pose else 0.0,
            'robot_y': float(y) if robot_pose else 0.0,
            'robot_yaw': float(yaw) if robot_pose else 0.0
        }
        
        with open(str(self.output_dir / 'map_latest.yaml'), 'w') as f:
            yaml.dump(map_yaml, f, default_flow_style=False)
        
        self.last_save_time = current_time

def main():
    rclpy.init()
    node = MapSaver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
