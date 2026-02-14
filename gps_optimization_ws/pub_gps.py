#!/usr/bin/env python3
"""
ROS 2 GPS Parser Node
Reads NMEA data from serial port and publishes sensor_msgs/NavSatFix messages.

Usage:
    ros2 run <package_name> pub_gps.py
    
Or standalone:
    python3 pub_gps.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
import serial
import threading
import time
from typing import Optional

try:
    import pynmea2
    HAS_PYNMEA2 = True
except ImportError:
    HAS_PYNMEA2 = False
    print("WARNING: pynmea2 not found. Using basic parsing. Install with: pip install pynmea2 --break-system-packages")


class GPSParserNode(Node):
    """ROS 2 node for parsing NMEA GPS data and publishing NavSatFix messages."""
    
    def __init__(self):
        super().__init__('gps_parser_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', './gps_filtered')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('reconnect_interval', 5.0)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        
        # Publisher
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # Serial connection or file
        self.serial_conn = None
        self.is_file_mode = False
        self.running = True
        self.last_valid_nmea = None
        self.eof_reached = False  # Track if we hit EOF
        
        # Start serial reader thread
        self.reader_thread = threading.Thread(target=self._serial_reader_thread, daemon=True)
        self.reader_thread.start()
        
        # Timer for re-publishing last data when EOF reached
        self.timer = self.create_timer(0.5, self._republish_if_eof)  # 2Hz
        
        # Last known GPS data
        self.last_gga_data = None
        self.last_rmc_data = None
        
        self.get_logger().info(f'GPS Parser Node started')
        self.get_logger().info(f'Port: {self.serial_port}, Baud: {self.baudrate}')
        
        if not HAS_PYNMEA2:
            self.get_logger().warn('pynmea2 library not found. Using basic parsing.')

    def _connect_serial(self) -> bool:
        """Attempt to connect to serial port or open file."""
        try:
            if self.serial_conn and hasattr(self.serial_conn, 'is_open') and self.serial_conn.is_open:
                self.serial_conn.close()
            
            import os
            import stat
            
            if os.path.exists(self.serial_port):
                mode = os.stat(self.serial_port).st_mode
                
                # Regular file
                if stat.S_ISREG(mode) or stat.S_ISFIFO(mode):
                    self.serial_conn = open(self.serial_port, 'r')
                    self.is_file_mode = True
                    self.get_logger().info(f'Opened file: {self.serial_port}')
                    return True
                # Serial port
                elif stat.S_ISCHR(mode):
                    self.serial_conn = serial.Serial(
                        port=self.serial_port,
                        baudrate=self.baudrate,
                        timeout=1.0
                    )
                    self.is_file_mode = False
                    self.get_logger().info(f'Connected to serial port: {self.serial_port}')
                    return True
            else:
                # Try as serial port
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baudrate,
                    timeout=1.0
                )
                self.is_file_mode = False
                self.get_logger().info(f'Connected to {self.serial_port}')
                return True
                
        except Exception as e:
            self.get_logger().error(f'Failed to connect to {self.serial_port}: {e}')
            return False

    def _serial_reader_thread(self):
        """Background thread for reading serial data or file."""
        while self.running:
            # Try to connect if not connected
            if not self.serial_conn:
                if not self._connect_serial():
                    self.get_logger().warn(f'Retrying connection in {self.reconnect_interval}s...')
                    threading.Event().wait(self.reconnect_interval)
                    continue
            
            try:
                # Read line based on mode
                if self.is_file_mode:
                    line = self.serial_conn.readline()
                    if not line:  # EOF reached
                        if not self.eof_reached:
                            self.get_logger().info(f'End of file reached. Last NMEA: {self.last_valid_nmea[:50] if self.last_valid_nmea else "None"}...')
                            self.get_logger().info('Will keep publishing last GPS data at 2Hz...')
                            self.eof_reached = True
                        # Just wait, timer will handle re-publishing
                        threading.Event().wait(1.0)
                        continue
                    line = line.strip()
                else:
                    # Serial port mode
                    if not self.serial_conn.is_open:
                        self.serial_conn = None
                        continue
                    
                    if self.serial_conn.in_waiting > 0:
                        line = self.serial_conn.readline().decode('ascii', errors='ignore').strip()
                    else:
                        continue
                
                # Extract and parse valid NMEA sentences directly
                if '$' in line:
                    dollar_idx = line.find('$')
                    clean_line = line[dollar_idx:]
                    
                    if clean_line.startswith('$') and ',' in clean_line and 15 < len(clean_line) < 200:
                        if 'GGA' in clean_line or 'RMC' in clean_line:
                            self.last_valid_nmea = clean_line
                            self.get_logger().info(f'Read NMEA: {clean_line[:60]}...')
                            # Parse and publish immediately
                            self._parse_nmea_sentence(clean_line)
                        
            except Exception as e:
                self.get_logger().error(f'Read error: {e}')
                if self.serial_conn:
                    self.serial_conn.close()
                    self.serial_conn = None
                time.sleep(1.0)

    def _republish_if_eof(self):
        """Re-publish last GPS data if EOF reached."""
        if self.eof_reached and (self.last_gga_data or self.last_rmc_data):
            self.get_logger().info('Re-publishing last GPS data')
            self._publish_navsat_fix()

    def _parse_nmea_sentence(self, sentence: str):
        """Parse NMEA sentence and publish NavSatFix if complete data available."""
        if HAS_PYNMEA2:
            self._parse_with_pynmea2(sentence)
        else:
            self._parse_basic(sentence)

    def _parse_with_pynmea2(self, sentence: str):
        """Parse using pynmea2 library."""
        try:
            msg = pynmea2.parse(sentence)
            
            # Handle GGA messages
            if isinstance(msg, pynmea2.types.talker.GGA):
                if msg.latitude is not None and msg.longitude is not None:
                    self.last_gga_data = msg
                    self._publish_navsat_fix()
            
            # Handle RMC messages
            elif isinstance(msg, pynmea2.types.talker.RMC):
                if msg.latitude is not None and msg.longitude is not None:
                    self.last_rmc_data = msg
                    self._publish_navsat_fix()
                    
        except Exception as e:
            self.get_logger().debug(f'Parse error: {e}')

    def _parse_basic(self, sentence: str):
        """Basic parsing without pynmea2 (fallback)."""
        parts = sentence.split(',')
        
        if len(parts) < 2:
            return
        
        msg_type = parts[0]
        
        # Parse GGA
        if 'GGA' in msg_type and len(parts) >= 15:
            self.last_gga_data = {
                'lat': parts[2],
                'lat_dir': parts[3],
                'lon': parts[4],
                'lon_dir': parts[5],
                'quality': int(parts[6]) if parts[6] else 0,
                'num_sats': int(parts[7]) if parts[7] else 0,
                'altitude': float(parts[9]) if parts[9] else 0.0,
            }
            self._publish_navsat_fix()

    def _convert_nmea_to_decimal(self, coord: str, direction: str) -> float:
        """Convert NMEA coordinate format (DDMM.MMMM) to decimal degrees."""
        if not coord or len(coord) < 4:
            return 0.0
        
        # Determine if latitude (DDMM) or longitude (DDDMM)
        if len(coord.split('.')[0]) <= 4:  # Latitude
            degrees = float(coord[:2])
            minutes = float(coord[2:])
        else:  # Longitude
            degrees = float(coord[:3])
            minutes = float(coord[3:])
        
        decimal = degrees + minutes / 60.0
        
        if direction in ['S', 'W']:
            decimal = -decimal
        
        return decimal

    def _publish_navsat_fix(self):
        """Publish NavSatFix message from accumulated GPS data."""
        if HAS_PYNMEA2:
            self._publish_with_pynmea2()
        else:
            self._publish_basic()

    def _publish_with_pynmea2(self):
        """Publish using pynmea2 parsed data."""
        # Prefer GGA data (has altitude and quality)
        if not self.last_gga_data and not self.last_rmc_data:
            return
        
        msg = NavSatFix()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Position - prefer GGA
        if self.last_gga_data:
            # pynmea2 already provides decimal degrees as float
            try:
                msg.latitude = float(self.last_gga_data.latitude) if self.last_gga_data.latitude is not None else 0.0
            except:
                msg.latitude = 0.0
            
            try:
                msg.longitude = float(self.last_gga_data.longitude) if self.last_gga_data.longitude is not None else 0.0
            except:
                msg.longitude = 0.0
            
            try:
                msg.altitude = float(self.last_gga_data.altitude) if self.last_gga_data.altitude is not None else 0.0
            except:
                msg.altitude = 0.0
            
            # Status from GGA quality
            quality = self.last_gga_data.gps_qual if self.last_gga_data.gps_qual is not None else 0
            try:
                quality_int = int(quality)
            except:
                quality_int = 0
            msg.status.status = self._map_gps_quality_to_status(quality_int)
            
            # Covariance estimation
            hdop = getattr(self.last_gga_data, 'horizontal_dil', None)
            if hdop is not None:
                try:
                    hdop_val = float(hdop)
                    if hdop_val > 0:
                        variance = (hdop_val * 2.0) ** 2
                    else:
                        variance = 25.0
                except:
                    variance = 25.0
            else:
                variance = 25.0
            
        elif self.last_rmc_data:
            try:
                msg.latitude = float(self.last_rmc_data.latitude) if self.last_rmc_data.latitude is not None else 0.0
            except:
                msg.latitude = 0.0
            
            try:
                msg.longitude = float(self.last_rmc_data.longitude) if self.last_rmc_data.longitude is not None else 0.0
            except:
                msg.longitude = 0.0
            
            msg.altitude = 0.0
            
            # Status from RMC validity
            if hasattr(self.last_rmc_data, 'status') and self.last_rmc_data.status == 'A':
                msg.status.status = NavSatStatus.STATUS_FIX
            else:
                msg.status.status = NavSatStatus.STATUS_NO_FIX
            
            variance = 25.0
        
        # Covariance matrix (diagonal only)
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        msg.position_covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance * 2.0
        ]
        
        # Service
        msg.status.service = NavSatStatus.SERVICE_GPS
        
        # Publish
        self.gps_pub.publish(msg)
        self.get_logger().info(f'GPS -> lat: {msg.latitude:.6f}, lon: {msg.longitude:.6f}, alt: {msg.altitude:.1f}m, status: {msg.status.status}')

    def _publish_basic(self):
        """Publish using basic parsed data."""
        if not self.last_gga_data:
            return
        
        msg = NavSatFix()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Position
        msg.latitude = self._convert_nmea_to_decimal(
            self.last_gga_data['lat'],
            self.last_gga_data['lat_dir']
        )
        msg.longitude = self._convert_nmea_to_decimal(
            self.last_gga_data['lon'],
            self.last_gga_data['lon_dir']
        )
        msg.altitude = self.last_gga_data['altitude']
        
        # Status
        msg.status.status = self._map_gps_quality_to_status(self.last_gga_data['quality'])
        msg.status.service = NavSatStatus.SERVICE_GPS
        
        # Covariance
        num_sats = self.last_gga_data['num_sats']
        variance = (10.0 / max(num_sats, 4)) ** 2
        
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        msg.position_covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance * 2.0
        ]
        
        # Publish
        self.gps_pub.publish(msg)
        self.get_logger().info(f'GPS -> lat: {msg.latitude:.6f}, lon: {msg.longitude:.6f}, alt: {msg.altitude:.1f}m, status: {msg.status.status}')

    def _map_gps_quality_to_status(self, quality: int) -> int:
        """
        Map GPS quality indicator to NavSatStatus.
        
        GGA Quality indicators:
        0 = Invalid
        1 = GPS fix (SPS)
        2 = DGPS fix
        4 = RTK fixed
        5 = RTK float
        """
        quality_map = {
            0: NavSatStatus.STATUS_NO_FIX,
            1: NavSatStatus.STATUS_FIX,
            2: NavSatStatus.STATUS_SBAS_FIX,
            4: NavSatStatus.STATUS_GBAS_FIX,  # RTK fixed
            5: NavSatStatus.STATUS_GBAS_FIX,  # RTK float
        }
        return quality_map.get(quality, NavSatStatus.STATUS_NO_FIX)

    def destroy_node(self):
        """Clean shutdown."""
        self.running = False
        if self.serial_conn:
            if self.is_file_mode:
                self.serial_conn.close()
            elif hasattr(self.serial_conn, 'is_open') and self.serial_conn.is_open:
                self.serial_conn.close()
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSParserNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()