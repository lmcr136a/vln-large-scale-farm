#!/usr/bin/env python3
import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class FakeGPSPublisher(Node):
    def __init__(self):
        super().__init__("fake_gps_publisher")

        # Parameters (easy to tweak without touching code)
        self.declare_parameter("topic", "/gps/fix")
        self.declare_parameter("frame_id", "gps_link")
        self.declare_parameter("rate_hz", 5.0)

        # Base location (pick any reasonable lat/lon)
        self.declare_parameter("base_lat", 37.4220)
        self.declare_parameter("base_lon", -122.0841)
        self.declare_parameter("base_alt", 10.0)

        # Motion + noise
        self.declare_parameter("radius_deg", 0.00001)   # ~1.1m at equator per 1e-5 deg
        self.declare_parameter("angular_speed", 0.2)    # radians per tick-ish
        self.declare_parameter("noise_deg", 0.000002)   # ~0.2m–0.3m noise
        self.declare_parameter("noise_alt_m", 0.3)
        self.declare_parameter("dropout_prob", 0.0)     # set to 0.05 to simulate GPS loss

        self.topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.base_lat = float(self.get_parameter("base_lat").value)
        self.base_lon = float(self.get_parameter("base_lon").value)
        self.base_alt = float(self.get_parameter("base_alt").value)

        self.radius_deg = float(self.get_parameter("radius_deg").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.noise_deg = float(self.get_parameter("noise_deg").value)
        self.noise_alt_m = float(self.get_parameter("noise_alt_m").value)
        self.dropout_prob = float(self.get_parameter("dropout_prob").value)

        self.pub = self.create_publisher(NavSatFix, self.topic, 10)

        self.t = 0.0
        period = 1.0 / max(self.rate_hz, 0.1)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Publishing fake GPS on {self.topic} at {self.rate_hz} Hz "
            f"(base: {self.base_lat}, {self.base_lon})"
        )

    def _tick(self):
        # Optional: simulate GPS dropout (no fix)
        has_fix = random.random() >= self.dropout_prob

        self.t += self.angular_speed

        lat = self.base_lat + self.radius_deg * math.cos(self.t)
        lon = self.base_lon + self.radius_deg * math.sin(self.t)
        alt = self.base_alt

        # Add noise
        lat += random.gauss(0.0, self.noise_deg)
        lon += random.gauss(0.0, self.noise_deg)
        alt += random.gauss(0.0, self.noise_alt_m)

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.status.status = NavSatStatus.STATUS_FIX if has_fix else NavSatStatus.STATUS_NO_FIX

        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(alt)

        # Unknown covariance (simple + valid)
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        msg.position_covariance[0] = -1.0

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeGPSPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
