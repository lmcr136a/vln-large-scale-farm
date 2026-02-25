#!/usr/bin/env python3
"""
ROS 2 GPS Parser + Trajectory Visualizer Node
NMEA 파싱 후 NavSatFix 발행 및 RViz 궤적 마커 동시 처리
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import serial
import threading
import time
import math
from typing import Optional

try:
    import pynmea2
    HAS_PYNMEA2 = True
except ImportError:
    HAS_PYNMEA2 = False
    print("WARNING: pynmea2 not found. Using basic parsing. Install with: pip install pynmea2 --break-system-packages")

ROBOT_HEIGHT = 1.0

# UCLA fallback coordinates (used when GPS has no fix)
UCLA_LAT = 34.068928
UCLA_LON = -118.445181
UCLA_ALT = 71.0


class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('marker_topic', '/gps_trajectory_marker')
        self.declare_parameter('map_frame_id', 'map')

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        gps_topic = self.get_parameter('gps_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self.map_frame_id = self.get_parameter('map_frame_id').value

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.gps_pub = self.create_publisher(NavSatFix, gps_topic, sensor_qos)
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)

        # Serial state
        self.serial_conn = None
        self.is_file_mode = False
        self.running = True
        self.last_valid_nmea = None
        self.eof_reached = False
        self.last_gga_data = None
        self.last_rmc_data = None

        # Trajectory state
        self.origin_set = False
        self.origin_lat = 0.0
        self.origin_lon = 0.0
        self.origin_alt = 0.0
        self.trajectory_points = []

        # Start reader thread + EOF re-publish timer
        self.reader_thread = threading.Thread(target=self._serial_reader_thread, daemon=True)
        self.reader_thread.start()
        self.create_timer(0.5, self._republish_if_eof)

        self.get_logger().info(f'GPS Node started | Port: {self.serial_port}, Baud: {self.baudrate}')
        self.get_logger().info(f'Publishing -> GPS: {gps_topic}, Markers: {marker_topic}')
        if not HAS_PYNMEA2:
            self.get_logger().warn('pynmea2 not found. Using basic parsing.')

    # ── Serial / File I/O ──────────────────────────────────────────────────

    def _connect_serial(self) -> bool:
        try:
            if self.serial_conn and hasattr(self.serial_conn, 'is_open') and self.serial_conn.is_open:
                self.serial_conn.close()

            import os, stat
            if os.path.exists(self.serial_port):
                mode = os.stat(self.serial_port).st_mode
                if stat.S_ISREG(mode) or stat.S_ISFIFO(mode):
                    self.serial_conn = open(self.serial_port, 'r')
                    self.is_file_mode = True
                    self.get_logger().info(f'Opened file: {self.serial_port}')
                    return True
                elif stat.S_ISCHR(mode):
                    self.serial_conn = serial.Serial(port=self.serial_port, baudrate=self.baudrate, timeout=1.0)
                    self.is_file_mode = False
                    self.get_logger().info(f'Connected to serial port: {self.serial_port}')
                    return True
            else:
                self.serial_conn = serial.Serial(port=self.serial_port, baudrate=self.baudrate, timeout=1.0)
                self.is_file_mode = False
                self.get_logger().info(f'Connected to {self.serial_port}')
                return True
        except Exception as e:
            self.get_logger().error(f'Failed to connect to {self.serial_port}: {e}')
            return False

    def _serial_reader_thread(self):
        while self.running:
            if not self.serial_conn:
                if not self._connect_serial():
                    self.get_logger().warn(f'Retrying in {self.reconnect_interval}s...')
                    threading.Event().wait(self.reconnect_interval)
                    continue
            try:
                if self.is_file_mode:
                    line = self.serial_conn.readline()
                    if not line:
                        if not self.eof_reached:
                            self.get_logger().info('EOF reached. Re-publishing last GPS at 2Hz...')
                            self.eof_reached = True
                        threading.Event().wait(1.0)
                        continue
                    line = line.strip()
                else:
                    if not self.serial_conn.is_open:
                        self.serial_conn = None
                        continue
                    if self.serial_conn.in_waiting > 0:
                        line = self.serial_conn.readline().decode('ascii', errors='ignore').strip()
                    else:
                        continue

                if '$' in line:
                    dollar_idx = line.find('$')
                    clean_line = line[dollar_idx:]
                    if clean_line.startswith('$') and ',' in clean_line and 15 < len(clean_line) < 200:
                        if 'GGA' in clean_line or 'RMC' in clean_line:
                            self.last_valid_nmea = clean_line
                            self.get_logger().info(f'NMEA: {clean_line[:60]}...')
                            self._parse_nmea_sentence(clean_line)

            except Exception as e:
                self.get_logger().error(f'Read error: {e}')
                if self.serial_conn:
                    self.serial_conn.close()
                    self.serial_conn = None
                time.sleep(1.0)

    def _republish_if_eof(self):
        if self.eof_reached and (self.last_gga_data or self.last_rmc_data):
            self.get_logger().info('Re-publishing last GPS data')
            self._publish_navsat_fix()

    # ── NMEA Parsing ───────────────────────────────────────────────────────

    def _parse_nmea_sentence(self, sentence: str):
        if HAS_PYNMEA2:
            self._parse_with_pynmea2(sentence)
        else:
            self._parse_basic(sentence)

    def _parse_with_pynmea2(self, sentence: str):
        try:
            msg = pynmea2.parse(sentence)
            if isinstance(msg, pynmea2.types.talker.GGA):
                # Store regardless of fix quality — fallback will handle no-fix case
                self.last_gga_data = msg
                self._publish_navsat_fix()
            elif isinstance(msg, pynmea2.types.talker.RMC):
                self.last_rmc_data = msg
                self._publish_navsat_fix()
        except Exception as e:
            self.get_logger().error(f'Parse error: {e}')

    def _parse_basic(self, sentence: str):
        parts = sentence.split(',')
        if len(parts) < 2:
            return
        if 'GGA' in parts[0] and len(parts) >= 15:
            self.last_gga_data = {
                'lat': parts[2], 'lat_dir': parts[3],
                'lon': parts[4], 'lon_dir': parts[5],
                'quality': int(parts[6]) if parts[6] else 0,
                'num_sats': int(parts[7]) if parts[7] else 0,
                'altitude': float(parts[9]) if parts[9] else 0.0,
            }
            self._publish_navsat_fix()

    def _convert_nmea_to_decimal(self, coord: str, direction: str) -> float:
        if not coord or len(coord) < 4:
            return 0.0
        if len(coord.split('.')[0]) <= 4:
            degrees, minutes = float(coord[:2]), float(coord[2:])
        else:
            degrees, minutes = float(coord[:3]), float(coord[3:])
        decimal = degrees + minutes / 60.0
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal

    def _map_gps_quality_to_status(self, quality: int) -> int:
        return {
            0: NavSatStatus.STATUS_NO_FIX,
            1: NavSatStatus.STATUS_FIX,
            2: NavSatStatus.STATUS_SBAS_FIX,
            4: NavSatStatus.STATUS_GBAS_FIX,
            5: NavSatStatus.STATUS_GBAS_FIX,
        }.get(quality, NavSatStatus.STATUS_NO_FIX)

    # ── NavSatFix Publishing ───────────────────────────────────────────────

    def _publish_navsat_fix(self):
        if HAS_PYNMEA2:
            self._publish_with_pynmea2()
        else:
            self._publish_basic()

    def _publish_with_pynmea2(self):
        if not self.last_gga_data and not self.last_rmc_data:
            return

        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.status.service = NavSatStatus.SERVICE_GPS

        if self.last_gga_data:
            try: lat = float(self.last_gga_data.latitude)
            except: lat = 0.0
            try: lon = float(self.last_gga_data.longitude)
            except: lon = 0.0
            try: quality_int = int(self.last_gga_data.gps_qual or 0)
            except: quality_int = 0

            # Use UCLA fallback when no valid fix
            if lat == 0.0 or lon == 0.0 or quality_int == 0:
                msg.latitude = UCLA_LAT
                msg.longitude = UCLA_LON
                msg.altitude = UCLA_ALT
                msg.status.status = NavSatStatus.STATUS_NO_FIX
                variance = 9999.0
                self.get_logger().warn('No GPS fix — publishing UCLA fallback coordinates')
            else:
                msg.latitude = lat
                msg.longitude = lon
                try: msg.altitude = float(self.last_gga_data.altitude) or 0.0
                except: msg.altitude = 0.0
                msg.status.status = self._map_gps_quality_to_status(quality_int)
                hdop = getattr(self.last_gga_data, 'horizontal_dil', None)
                try: variance = (float(hdop) * 2.0) ** 2 if hdop and float(hdop) > 0 else 25.0
                except: variance = 25.0
        else:
            try: lat = float(self.last_rmc_data.latitude)
            except: lat = 0.0
            try: lon = float(self.last_rmc_data.longitude)
            except: lon = 0.0
            is_active = hasattr(self.last_rmc_data, 'status') and self.last_rmc_data.status == 'A'

            if lat == 0.0 or lon == 0.0 or not is_active:
                msg.latitude = UCLA_LAT
                msg.longitude = UCLA_LON
                msg.altitude = UCLA_ALT
                msg.status.status = NavSatStatus.STATUS_NO_FIX
                variance = 9999.0
                self.get_logger().warn('No GPS fix — publishing UCLA fallback coordinates')
            else:
                msg.latitude = lat
                msg.longitude = lon
                msg.altitude = 0.0
                msg.status.status = NavSatStatus.STATUS_FIX
                variance = 25.0

        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        msg.position_covariance = [float(variance), 0.0, 0.0, 0.0, float(variance), 0.0, 0.0, 0.0, float(variance) * 2.0]

        try:
            self.gps_pub.publish(msg)
            self.get_logger().info(f'NavSatFix -> lat: {msg.latitude:.6f}, lon: {msg.longitude:.6f}, alt: {msg.altitude:.1f}m')
        except Exception as pub_err:
            self.get_logger().error(f'PUBLISH FAILED: {pub_err}')

        if msg.status.status >= 0:
            self._update_trajectory(msg.latitude, msg.longitude, msg.altitude)

    def _publish_basic(self):
        if not self.last_gga_data:
            return

        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.latitude = self._convert_nmea_to_decimal(self.last_gga_data['lat'], self.last_gga_data['lat_dir'])
        msg.longitude = self._convert_nmea_to_decimal(self.last_gga_data['lon'], self.last_gga_data['lon_dir'])
        msg.altitude = self.last_gga_data['altitude']
        msg.status.status = self._map_gps_quality_to_status(self.last_gga_data['quality'])
        msg.status.service = NavSatStatus.SERVICE_GPS

        num_sats = self.last_gga_data['num_sats']
        variance = (10.0 / max(num_sats, 4)) ** 2
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        msg.position_covariance = [float(variance), 0.0, 0.0, 0.0, float(variance), 0.0, 0.0, 0.0, float(variance) * 2.0]

        self.gps_pub.publish(msg)
        self.get_logger().info(f'NavSatFix -> lat: {msg.latitude:.6f}, lon: {msg.longitude:.6f}, alt: {msg.altitude:.1f}m')

        if msg.status.status >= 0:
            self._update_trajectory(msg.latitude, msg.longitude, msg.altitude)

    # ── Trajectory / Marker ────────────────────────────────────────────────

    def _update_trajectory(self, lat, lon, alt):
        if not self.origin_set:
            self.origin_lat, self.origin_lon, self.origin_alt = lat, lon, alt
            self.origin_set = True
            self.get_logger().info(f'Origin set: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.2f}')
            return

        e, n, _ = self._lla_to_enu(lat, lon, alt, self.origin_lat, self.origin_lon, self.origin_alt)

        pt = Point()
        pt.x, pt.y, pt.z = e, n, ROBOT_HEIGHT
        self.trajectory_points.append(pt)

        self._publish_markers()
        self.get_logger().info(f'ENU -> E={e:.2f}, N={n:.2f} (Total: {len(self.trajectory_points)})')

    def _lla_to_enu(self, lat, lon, alt, lat0, lon0, alt0):
        dlat = lat - lat0
        dlon = lon - lon0
        lat_avg = math.radians((lat + lat0) / 2.0)
        e = dlon * 111320.0 * math.cos(lat_avg)
        n = dlat * 110540.0
        u = alt - alt0
        return e, n, u

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()

        # Line strip trajectory
        m = Marker()
        m.header.frame_id = self.map_frame_id
        m.header.stamp = now
        m.ns = "gps_trajectory"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.5
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
        m.points = self.trajectory_points
        self.marker_pub.publish(m)

        # Current position sphere
        if self.trajectory_points:
            cm = Marker()
            cm.header.frame_id = self.map_frame_id
            cm.header.stamp = now
            cm.ns = "gps_current"
            cm.id = 1
            cm.type = Marker.SPHERE
            cm.action = Marker.ADD
            cm.pose.position = self.trajectory_points[-1]
            cm.scale.x = cm.scale.y = cm.scale.z = 2.0
            cm.color.r, cm.color.g, cm.color.b, cm.color.a = 0.0, 1.0, 0.0, 1.0
            self.marker_pub.publish(cm)

    # ── Shutdown ───────────────────────────────────────────────────────────

    def destroy_node(self):
        self.running = False
        if self.serial_conn:
            try: self.serial_conn.close()
            except: pass
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=2.0)
        self.get_logger().info(f'Shutdown. Total GPS points: {len(self.trajectory_points)}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()