#!/usr/bin/env python3
"""
GLIM converter nodes:
  1. Livox CustomMsg -> PointCloud2 (with per-point time offsets)
  2. sensor_msgs/NavSatFix -> geometry_msgs/PoseWithCovarianceStamped (ENU local coords)
  3. /glim_ros/map QoS bridge (TRANSIENT_LOCAL -> VOLATILE) for octomap_server

Usage:
  python3 glim_converters.py
  python3 glim_converters.py --ros-args \
    -p lidar_input_topic:=/livox/lidar \
    -p lidar_output_topic:=/livox/lidar_pointcloud2 \
    -p gps_input_topic:=/gps/fix \
    -p gps_output_topic:=/gps/fix_pose \
    -p map_input_topic:=/glim_ros/map \
    -p map_output_topic:=/glim_ros/map_volatile
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math
import numpy as np

from sensor_msgs.msg import NavSatFix, PointCloud2, PointField
from geometry_msgs.msg import PoseWithCovarianceStamped
from livox_ros_driver2.msg import CustomMsg


# WGS84 ellipsoid parameters
WGS84_A = 6378137.0
WGS84_E2 = 0.00669437999014


def geodetic_to_ecef(lat, lon, alt):
    lat_r = math.radians(lat)
    lon_r = math.radians(lon)
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat_r) ** 2)
    x = (N + alt) * math.cos(lat_r) * math.cos(lon_r)
    y = (N + alt) * math.cos(lat_r) * math.sin(lon_r)
    z = (N * (1 - WGS84_E2) + alt) * math.sin(lat_r)
    return x, y, z


def ecef_to_enu(ecef, origin_ecef, origin_lat, origin_lon):
    lat_r = math.radians(origin_lat)
    lon_r = math.radians(origin_lon)
    dx = ecef[0] - origin_ecef[0]
    dy = ecef[1] - origin_ecef[1]
    dz = ecef[2] - origin_ecef[2]
    e = -math.sin(lon_r) * dx + math.cos(lon_r) * dy
    n = (-math.sin(lat_r) * math.cos(lon_r) * dx
         - math.sin(lat_r) * math.sin(lon_r) * dy
         + math.cos(lat_r) * dz)
    u = (math.cos(lat_r) * math.cos(lon_r) * dx
         + math.cos(lat_r) * math.sin(lon_r) * dy
         + math.sin(lat_r) * dz)
    return e, n, u


class GlimConverters(Node):
    def __init__(self):
        super().__init__('glim_converters')

        # --- Parameters ---
        self.declare_parameter('lidar_input_topic',  '/livox/lidar')
        self.declare_parameter('lidar_output_topic', '/livox/lidar_pointcloud2')
        self.declare_parameter('gps_input_topic',    '/gps/fix')
        self.declare_parameter('gps_output_topic',   '/gps/fix_pose')
        self.declare_parameter('map_input_topic',    '/glim_ros/map')
        self.declare_parameter('map_output_topic',   '/glim_ros/map_volatile')

        lidar_in  = self.get_parameter('lidar_input_topic').value
        lidar_out = self.get_parameter('lidar_output_topic').value
        gps_in    = self.get_parameter('gps_input_topic').value
        gps_out   = self.get_parameter('gps_output_topic').value
        map_in    = self.get_parameter('map_input_topic').value
        map_out   = self.get_parameter('map_output_topic').value

        # --- LiDAR converter ---
        self.lidar_sub = self.create_subscription(CustomMsg, lidar_in, self.lidar_callback, 10)
        self.lidar_pub = self.create_publisher(PointCloud2, lidar_out, 10)

        # Preallocate fields list (same every time)
        self._fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='time',      offset=16, datatype=PointField.FLOAT32, count=1),
        ]

        # --- GPS converter ---
        self.gps_sub = self.create_subscription(NavSatFix, gps_in, self.gps_callback, 10)
        self.gps_pub = self.create_publisher(PoseWithCovarianceStamped, gps_out, 10)
        self.gps_origin_ecef = None
        self.gps_origin_lat  = None
        self.gps_origin_lon  = None

        # --- Map QoS bridge (TRANSIENT_LOCAL -> VOLATILE for octomap_server) ---
        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        volatile_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.map_sub = self.create_subscription(
            PointCloud2, map_in, self.map_callback, transient_qos)
        self.map_pub = self.create_publisher(PointCloud2, map_out, volatile_qos)

        self.get_logger().info(f'[LiDAR] {lidar_in} -> {lidar_out}')
        self.get_logger().info(f'[GPS]   {gps_in} -> {gps_out}')
        self.get_logger().info(f'[MAP]   {map_in} -> {map_out} (QoS bridge)')

    # -------------------------------------------------------------------------
    # LiDAR: CustomMsg -> PointCloud2 (numpy vectorized)
    # -------------------------------------------------------------------------
    def lidar_callback(self, msg: CustomMsg):
        n = msg.point_num
        if n == 0:
            return

        points = msg.points

        # Vectorized extraction using numpy
        arr = np.empty((n, 5), dtype=np.float32)
        arr[:, 0] = [p.x            for p in points]
        arr[:, 1] = [p.y            for p in points]
        arr[:, 2] = [p.z            for p in points]
        arr[:, 3] = [p.reflectivity for p in points]
        arr[:, 4] = [p.offset_time * 1e-9 for p in points]  # ns -> s

        pc2 = PointCloud2()
        pc2.header      = msg.header
        pc2.height      = 1
        pc2.width       = n
        pc2.fields      = self._fields
        pc2.is_bigendian = False
        pc2.point_step  = 20  # 5 * float32
        pc2.row_step    = 20 * n
        pc2.data        = arr.tobytes()
        pc2.is_dense    = False
        self.lidar_pub.publish(pc2)

    # -------------------------------------------------------------------------
    # GPS: NavSatFix -> PoseWithCovarianceStamped (ENU)
    # -------------------------------------------------------------------------
    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            return

        lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
        ecef = geodetic_to_ecef(lat, lon, alt)

        if self.gps_origin_ecef is None:
            self.gps_origin_ecef = ecef
            self.gps_origin_lat  = lat
            self.gps_origin_lon  = lon
            self.get_logger().info(
                f'[GPS] origin set: lat={lat:.6f} lon={lon:.6f} alt={alt:.2f}')

        e, n, u = ecef_to_enu(ecef, self.gps_origin_ecef,
                               self.gps_origin_lat, self.gps_origin_lon)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = e
        pose_msg.pose.pose.position.y = n
        pose_msg.pose.pose.position.z = u
        pose_msg.pose.pose.orientation.w = 1.0

        cov = [0.0] * 36
        if msg.position_covariance_type > 0:
            cov[0]  = msg.position_covariance[0]
            cov[1]  = msg.position_covariance[1]
            cov[2]  = msg.position_covariance[2]
            cov[6]  = msg.position_covariance[3]
            cov[7]  = msg.position_covariance[4]
            cov[8]  = msg.position_covariance[5]
            cov[12] = msg.position_covariance[6]
            cov[13] = msg.position_covariance[7]
            cov[14] = msg.position_covariance[8]
        else:
            cov[0] = cov[7] = cov[14] = 1.0

        pose_msg.pose.covariance = cov
        self.gps_pub.publish(pose_msg)

    # -------------------------------------------------------------------------
    # Map QoS bridge: TRANSIENT_LOCAL -> VOLATILE
    # -------------------------------------------------------------------------
    def map_callback(self, msg: PointCloud2):
        self.map_pub.publish(msg)


def main():
    rclpy.init()
    node = GlimConverters()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()