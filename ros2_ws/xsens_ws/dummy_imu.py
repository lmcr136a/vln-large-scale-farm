# fake_imu.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class FakeImu(Node):
    def __init__(self):
        super().__init__('fake_imu')
        self.pub = self.create_publisher(Imu, '/xsens/imu/data', 10)
        self.create_timer(0.005, self.publish)  # 200Hz

    def publish(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu'
        msg.linear_acceleration.z = 9.81
        self.pub.publish(msg)

rclpy.init()
rclpy.spin(FakeImu())