#!/usr/bin/env python3
"""
GPS Trajectory Plotter
간단한 GPS 궤적 시각화 스크립트
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math

class GPSTrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('gps_trajectory_plotter')
        
        # 파라미터
        self.declare_parameter('gps_topic', '/ublox_driver/receiver_lla')
        self.declare_parameter('marker_topic', '/gps_trajectory_marker')
        self.declare_parameter('frame_id', 'map')
        
        gps_topic = self.get_parameter('gps_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # GPS 구독
        self.gps_sub = self.create_subscription(
            NavSatFix,
            gps_topic,
            self.gps_callback,
            10
        )
        
        # Marker 발행
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)
        
        # 원점 설정
        self.origin_set = False
        self.origin_lat = 0.0
        self.origin_lon = 0.0
        self.origin_alt = 0.0
        
        # 궤적 저장
        self.trajectory_points = []
        
        self.get_logger().info(f'GPS Trajectory Plotter started')
        self.get_logger().info(f'  GPS topic: {gps_topic}')
        self.get_logger().info(f'  Marker topic: {marker_topic}')
        self.get_logger().info(f'Waiting for GPS data...')
    
    def gps_callback(self, msg):
        # GPS 상태 확인
        if msg.status.status < 0:
            return
        
        # 첫 GPS로 원점 설정
        if not self.origin_set:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude
            self.origin_set = True
            self.get_logger().info(
                f'Origin set: Lat={self.origin_lat:.6f}, '
                f'Lon={self.origin_lon:.6f}, Alt={self.origin_alt:.2f}'
            )
            return
        
        # LLA를 ENU로 변환
        e, n, u = self.lla_to_enu(
            msg.latitude, msg.longitude, msg.altitude,
            self.origin_lat, self.origin_lon, self.origin_alt
        )
        
        # 궤적에 추가
        point = Point()
        point.x = e
        point.y = n
        point.z = u
        self.trajectory_points.append(point)
        
        # Marker 발행
        self.publish_trajectory_marker()
        
        self.get_logger().info(
            f'GPS: E={e:.2f}, N={n:.2f}, U={u:.2f} '
            f'(Total points: {len(self.trajectory_points)})'
        )
    
    def lla_to_enu(self, lat, lon, alt, lat0, lon0, alt0):
        """LLA를 ENU 좌표로 변환 (간단한 평면 근사)"""
        # 위도 1도 ≈ 111km
        # 경도 1도 ≈ 111km × cos(위도)
        
        dlat = lat - lat0
        dlon = lon - lon0
        dalt = alt - alt0
        
        # 평균 위도에서의 경도 스케일
        lat_avg = math.radians((lat + lat0) / 2.0)
        
        # ENU 좌표 계산
        e = dlon * 111320.0 * math.cos(lat_avg)  # East
        n = dlat * 110540.0  # North
        u = dalt  # Up
        
        return e, n, u
    
    def publish_trajectory_marker(self):
        """RViz Marker로 궤적 발행"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gps_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # 선 설정
        marker.scale.x = 0.5  # 선 두께
        
        # 색상 (빨간색)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # 궤적 포인트 추가
        marker.points = self.trajectory_points
        
        # 발행
        self.marker_pub.publish(marker)
        
        # 현재 위치 마커도 발행
        if len(self.trajectory_points) > 0:
            current_marker = Marker()
            current_marker.header.frame_id = self.frame_id
            current_marker.header.stamp = self.get_clock().now().to_msg()
            current_marker.ns = "gps_current"
            current_marker.id = 1
            current_marker.type = Marker.SPHERE
            current_marker.action = Marker.ADD
            
            # 현재 위치
            current_marker.pose.position = self.trajectory_points[-1]
            
            # 크기
            current_marker.scale.x = 2.0
            current_marker.scale.y = 2.0
            current_marker.scale.z = 2.0
            
            # 색상 (녹색)
            current_marker.color.r = 0.0
            current_marker.color.g = 1.0
            current_marker.color.b = 0.0
            current_marker.color.a = 1.0
            
            self.marker_pub.publish(current_marker)

def main(args=None):
    rclpy.init(args=args)
    node = GPSTrajectoryPlotter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 전 통계 출력
        node.get_logger().info(f'Shutting down...')
        node.get_logger().info(f'Total GPS points collected: {len(node.trajectory_points)}')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()