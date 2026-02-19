#!/usr/bin/env python3
import serial
import pynmea2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GPSSerialPublisher(Node):
    def __init__(self):
        super().__init__("gps_serial_publisher")

        # Parameters
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 9600)          # many GPS are 9600; some are 115200
        self.declare_parameter("topic", "/gps/fix")
        self.declare_parameter("frame_id", "gps_link")
        self.declare_parameter("rate_hz", 10.0)       # node loop rate (serial read + publish)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.pub = self.create_publisher(NavSatFix, self.topic, 10)

        # Open serial
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
            self.get_logger().info(f"Opened GPS serial: {self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open {self.port}: {e}")
            raise

        # Track last good fix so we can publish consistently
        self.last_lat = None
        self.last_lon = None
        self.last_alt = None
        self.last_fix_ok = False

        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self._tick)

    def _tick(self):
        # Read a few lines each tick to keep up with incoming NMEA stream
        for _ in range(5):
            try:
                line = self.ser.readline().decode("ascii", errors="ignore").strip()
            except Exception:
                return

            if not line.startswith("$"):
                continue

            try:
                msg = pynmea2.parse(line)
            except Exception:
                continue

            # Prefer GGA for fix + altitude
            if isinstance(msg, pynmea2.types.talker.GGA):
                lat = msg.latitude
                lon = msg.longitude

                # gps_qual: 0 invalid; 1 GPS fix; 2 DGPS; others depend on receiver
                try:
                    fix_quality = int(msg.gps_qual) if msg.gps_qual not in (None, "") else 0
                except Exception:
                    fix_quality = 0

                alt = None
                try:
                    alt = float(msg.altitude) if msg.altitude not in (None, "") else None
                except Exception:
                    alt = None

                if lat != 0.0 and lon != 0.0 and fix_quality > 0:
                    self.last_lat = lat
                    self.last_lon = lon
                    self.last_alt = alt
                    self.last_fix_ok = True
                else:
                    self.last_fix_ok = False

            # RMC can also provide lat/lon even if GGA is missing sometimes
            elif isinstance(msg, pynmea2.types.talker.RMC):
                if getattr(msg, "status", "") == "A":  # Active
                    lat = msg.latitude
                    lon = msg.longitude
                    if lat != 0.0 and lon != 0.0:
                        self.last_lat = lat
                        self.last_lon = lon

        # Publish if we have any position at all
        if self.last_lat is None or self.last_lon is None:
            return

        out = NavSatFix()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id

        out.status.service = NavSatStatus.SERVICE_GPS
        out.status.status = NavSatStatus.STATUS_FIX if self.last_fix_ok else NavSatStatus.STATUS_NO_FIX

        out.latitude = float(self.last_lat)
        out.longitude = float(self.last_lon)
        out.altitude = float(self.last_alt) if self.last_alt is not None else float("nan")

        # Covariance unknown (-1 in first element is common to indicate unknown)
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        out.position_covariance[0] = -1.0

        self.pub.publish(out)

    def destroy_node(self):
        try:
            if hasattr(self, "ser") and self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = GPSSerialPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
