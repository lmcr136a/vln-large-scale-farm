#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import math

class GpsToOdom(Node):
    def __init__(self):
        super().__init__('gps_to_odom')
        self.sub = self.create_subscription(NavSatFix, '/gps/fix', self.callback, 10)
        self.pub = self.create_publisher(Odometry, '/odometry/gps', 10)
        self.origin_lat = None
        self.origin_lon = None
        
    def callback(self, msg):
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f'GPS origin: {self.origin_lat}, {self.origin_lon}')
        
        # 로컬 XY 변환
        R = 6378137.0
        x = R * math.radians(msg.longitude - self.origin_lon) * math.cos(math.radians(self.origin_lat))
        y = R * math.radians(msg.latitude - self.origin_lat)
        
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = msg.altitude
        odom.pose.covariance[0] = msg.position_covariance[0]
        odom.pose.covariance[7] = msg.position_covariance[4]
        odom.pose.covariance[14] = msg.position_covariance[8]
        
        self.pub.publish(odom)

rclpy.init()
node = GpsToOdom()
rclpy.spin(node)
