#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
import pynmea2

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # UCLA 기본 좌표
        self.default_lat = 34.0686081
        self.default_lon = -118.4438581
        self.default_alt = 105.0
        
        try:
            self.serial_port = serial.Serial(
                '/home/nahyeon/box/gps_filtered',
                9600,
                timeout=1
            )
            self.get_logger().info('GPS Publisher started')
        except Exception as e:
            self.get_logger().error(f'Failed to open port: {e}')
            return
        
        self.timer = self.create_timer(0.1, self.read_gps)
    
    def read_gps(self):
        try:
            line = self.serial_port.readline().decode('ascii', errors='ignore')
            
            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                try:
                    msg = pynmea2.parse(line)
                    
                    nav_msg = NavSatFix()
                    nav_msg.header.stamp = self.get_clock().now().to_msg()
                    nav_msg.header.frame_id = 'gps'
                    
                    # 위도/경도 확인 - 비어있으면 UCLA 좌표 사용
                    if msg.latitude and msg.longitude and msg.gps_qual > 0:
                        nav_msg.latitude = msg.latitude
                        nav_msg.longitude = msg.longitude
                        nav_msg.altitude = msg.altitude if msg.altitude else self.default_alt
                        nav_msg.status.status = NavSatStatus.STATUS_FIX
                        self.get_logger().info(f'GPS Fix: {msg.latitude:.6f}, {msg.longitude:.6f}')
                    else:
                        # UCLA 좌표 사용
                        nav_msg.latitude = self.default_lat
                        nav_msg.longitude = self.default_lon
                        nav_msg.altitude = self.default_alt
                        nav_msg.status.status = NavSatStatus.STATUS_NO_FIX
                        self.get_logger().warn('No GPS fix - using UCLA coordinates')
                    
                    nav_msg.status.service = NavSatStatus.SERVICE_GPS
                    nav_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                    nav_msg.position_covariance[0] = 1.0
                    nav_msg.position_covariance[4] = 1.0
                    nav_msg.position_covariance[8] = 1.0
                    
                    self.publisher.publish(nav_msg)
                    
                except pynmea2.ParseError as e:
                    self.get_logger().warn(f'Parse error: {e}')
                    
        except Exception as e:
            self.get_logger().warn(f'Read timeout: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()