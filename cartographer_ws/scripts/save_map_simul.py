#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageFilter
import time
import math
import yaml
import os
from pathlib import Path
from tf2_ros import Buffer, TransformListener

RED1 = (200, 60, 40)   # robot arrow
BLUE1 = (20, 30, 140)   # robot body
BLUE2 = (55, 150, 185)  # light, for trajectory
ROBOT_INFO_TEXT_COL = (255, 255, 0)


class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        
        self.workspace_dir = Path(__file__).resolve().parent.parent
        self.output_dir = self.workspace_dir / 'output'
        self.figures_dir = self.output_dir / 'figures'
        
        self.output_dir.mkdir(exist_ok=True)
        self.figures_dir.mkdir(exist_ok=True)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        self.create_subscription(
            MarkerArray,
            '/trajectory_node_list',
            self.trajectory_callback,
            10)
        
        self.last_save_time = 0
        self.save_interval = 1
        self.trajectory_nodes = []
        self.robot_pose = None
        self.prev_map_info = None
        self.save_count = 0
        
    def trajectory_callback(self, msg):
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
        
        yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        
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
        
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        img = np.zeros((height, width), dtype=np.uint8)
        
        unknown_mask = (data == -1)
        free_mask = (data >= 0) & (data < 50)
        occupied_mask = (data >= 50)
        
        img[unknown_mask] = 128
        img[free_mask] = 255 - (data[free_mask] * 5.1).astype(np.uint8)
        img[occupied_mask] = (255 - (data[occupied_mask] * 2.55)).astype(np.uint8)
        
        img = np.flipud(img)
        
        im = Image.fromarray(img, mode='L')
        im = im.filter(ImageFilter.GaussianBlur(radius=0.5))
        
        im = im.convert('RGB')
        draw = ImageDraw.Draw(im, 'RGBA')
        
        small_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
    
        
        grid_spacing_world = 5.0
        grid_spacing_cells = int(grid_spacing_world / resolution)
        
        for i in range(0, width, grid_spacing_cells):
            draw.line([(i, 0), (i, height-1)], fill=(100, 100, 100, 100), width=1)
            world_x = origin_x + i * resolution
            draw.text((i+2, 5), f'{world_x:.0f}m', fill=(100, 100, 100), font=small_font)
        
        for j in range(0, height, grid_spacing_cells):
            draw.line([(0, j), (width-1, j)], fill=(100, 100, 100, 100), width=1)
            world_y = origin_y + (height - 1 - j) * resolution
            draw.text((5, j+2), f'{world_y:.0f}m', fill=(100, 100, 100), font=small_font)
        
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
            draw.text((origin_img_x + 5, origin_img_y - 15), 
                     'World (0,0)', fill=(255, 0, 0), font=small_font)

        for x, y in self.trajectory_nodes:
            img_x = int((x - origin_x) / resolution)
            img_y = height - 1 - int((y - origin_y) / resolution)
            
            lw = 1  # line width
            if 0 <= img_x < width and 0 <= img_y < height:
                draw.ellipse([img_x-lw, img_y-lw, img_x+lw, img_y+lw], fill=BLUE2)
        
        if robot_pose:
            x, y, yaw = robot_pose
            
            img_x = int((x - origin_x) / resolution)
            img_y = height - 1 - int((y - origin_y) / resolution)
            
            if 0 <= img_x < width and 0 <= img_y < height:                
                robot_size = 10
                arrow_length = robot_size * 2.1
                arrow_width = lw
                head_length = robot_size * 0.75
                
                draw.ellipse([img_x-robot_size, img_y-robot_size, 
                             img_x+robot_size, img_y+robot_size], 
                            fill=BLUE1, outline=RED1, width=arrow_width)
                
                end_x = img_x + arrow_length * math.cos(yaw)
                end_y = img_y - arrow_length * math.sin(yaw)
                
                draw.line([(img_x, img_y), (end_x, end_y)], 
                         fill=RED1, width=arrow_width+1)
                
                arrow_angle = 30 * math.pi / 180
                
                left_x = end_x - head_length * math.cos(yaw - arrow_angle)
                left_y = end_y + head_length * math.sin(yaw - arrow_angle)
                right_x = end_x - head_length * math.cos(yaw + arrow_angle)
                right_y = end_y + head_length * math.sin(yaw + arrow_angle)
                
                draw.polygon([(end_x, end_y), (left_x, left_y), (right_x, right_y)], 
                           fill=RED1)
                
                info_text = f"Robot: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°) | Nodes: {len(self.trajectory_nodes)}"
                bbox = draw.textbbox((0, 0), info_text, font=small_font)
                text_width = bbox[2] - bbox[0]
                text_height = bbox[3] - bbox[1]
                draw.rectangle([(10, 10), (10 + text_width + 10, 10 + text_height + 10)], 
                              fill=(0, 0, 0, 180))
                draw.text((15, 15), info_text, fill=ROBOT_INFO_TEXT_COL, font=small_font)
        
        self.save_count += 1
        im.save(str(self.figures_dir / f'building_{self.save_count}.png'))
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