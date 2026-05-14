#!/usr/bin/env python3
"""
rtk_gps_node.py
  - Bridges RTCM corrections from radio → F9P GNSS receiver
  - Parses NMEA GGA from F9P
  - Publishes sensor_msgs/NavSatFix on /gps/fix at 10 Hz (RTK Fixed only)
  - Prints status every 2 s
"""

import threading
import struct
import time
import math

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

# ── Port config ────────────────────────────────────────────
RADIO_PORT = '/dev/ttyUSB0'   # radio (RTCM source)
F9P_PORT   = '/dev/ttyACM0'   # u-blox F9P GNSS receiver
BAUD_RADIO = 57600
BAUD_F9P   = 115200

PUBLISH_HZ      = 10
STATUS_INTERVAL = 2.0   # seconds between console prints


# ── UBX helpers ───────────────────────────────────────────
def _ubx_checksum(msg: bytes) -> bytes:
    ck_a = ck_b = 0
    for b in msg:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])

def _ubx_packet(cls: int, mid: int, payload: bytes = b'') -> bytes:
    msg = bytes([cls, mid]) + struct.pack('<H', len(payload)) + payload
    return b'\xb5\x62' + msg + _ubx_checksum(msg)

def _configure_f9p(ser: serial.Serial) -> None:
    time.sleep(0.3)
    cfg_keys = [
        (0x10730004, 1),  # UART1INPROT-RTCM3X
        (0x10730001, 1),  # UART1INPROT-UBX
        (0x10730002, 1),  # UART1INPROT-NMEA
        (0x10740002, 1),  # UART1OUTPROT-NMEA
        (0x10770004, 1),  # USBINPROT-RTCM3X  ← USB로 RTCM 받기
        (0x10770001, 1),  # USBINPROT-UBX
        (0x10770002, 1),  # USBINPROT-NMEA
        (0x10780002, 1),  # USBOUTPROT-NMEA
        (0x10310021, 1),  # SIGNAL-GPS_ENA
        (0x10310025, 1),  # SIGNAL-GLO_ENA
        (0x10310031, 1),  # SIGNAL-GAL_ENA
        (0x10310022, 1),  # SIGNAL-GPS_L1CA_ENA
        (0x10310024, 1),  # SIGNAL-GPS_L2C_ENA
    ]
    payload = struct.pack('<BBH', 0, 1, 0)
    for key, val in cfg_keys:
        payload += struct.pack('<IB', key, val)
    ser.write(_ubx_packet(0x06, 0x8A, payload))
    time.sleep(0.2)
    ser.read(256)


# ── NMEA helpers ──────────────────────────────────────────
def _nmea_to_decimal(value: str, direction: str):
    try:
        dot = value.index('.')
        deg = float(value[:dot - 2])
        minutes = float(value[dot - 2:])
        dec = deg + minutes / 60.0
        if direction in ('S', 'W'):
            dec = -dec
        return dec
    except Exception:
        return None

# GGA fix quality → (NavSatStatus fix type, is_rtk_fixed)
_GGA_FIX_MAP = {
    0: (NavSatStatus.STATUS_NO_FIX,   False),
    1: (NavSatStatus.STATUS_FIX,      False),
    2: (NavSatStatus.STATUS_SBAS_FIX, False),
    4: (NavSatStatus.STATUS_GBAS_FIX, True),   # RTK Fixed
    5: (NavSatStatus.STATUS_GBAS_FIX, False),  # RTK Float
}


class RtkGpsNode(Node):
    def __init__(self):
        super().__init__('rtk_gps_node')

        self._pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)
        self._status_timer = self.create_timer(STATUS_INTERVAL, self._print_status)

        self._state_lock = threading.Lock()
        self._state = {
            'lat': None, 'lon': None, 'alt': None,
            'hdop': 99.9, 'sv': 0,
            'rtk_fixed': False, 'status_str': 'No Fix',
            'rtcm_count': 0,
        }

        try:
            self._radio = serial.Serial(RADIO_PORT, BAUD_RADIO, timeout=0.1)
            self._f9p   = serial.Serial(F9P_PORT,   BAUD_F9P,   timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open serial port: {e}')
            raise

        _configure_f9p(self._f9p)
        self.get_logger().info('F9P configured. Starting threads...')

        threading.Thread(target=self._bridge_thread, daemon=True).start()
        threading.Thread(target=self._nmea_thread,   daemon=True).start()

    # ── RTCM bridge: radio → F9P raw passthrough ─────────
    def _bridge_thread(self):
        while rclpy.ok():
            try:
                data = self._radio.read(4096)
            except serial.SerialException:
                print('[RADIO] disconnected, retrying in 2s...')
                time.sleep(2.0)
                try:
                    self._radio.close()
                    self._radio = serial.Serial(RADIO_PORT, BAUD_RADIO, timeout=0.1)
                    print('[RADIO] reconnected')
                except Exception as e:
                    print(f'[RADIO] reconnect failed: {e}')
                continue

            if not data:
                continue

            self._f9p.write(data)
            with self._state_lock:
                self._state['rtcm_count'] += data.count(0xD3)

    # ── NMEA reader: F9P → state ──────────────────────────
    def _nmea_thread(self):
        buf = b''
        while rclpy.ok():
            chunk = self._f9p.read(256)
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self._parse_gga(line.decode('ascii', errors='ignore').strip())

    def _parse_gga(self, line: str):
        try:
            if not line.startswith('$'):
                return
            parts = line.split(',')
            if parts[0] not in ('$GNGGA', '$GPGGA') or len(parts) <= 9:
                return

            lat  = _nmea_to_decimal(parts[2], parts[3])
            lon  = _nmea_to_decimal(parts[4], parts[5])
            fix  = int(parts[6]) if parts[6] else 0
            sv   = int(parts[7]) if parts[7] else 0
            hdop = float(parts[8]) if parts[8] else 99.9
            alt  = float(parts[9]) if parts[9] else None

            if lat is None or lon is None or alt is None:
                return

            _fix_status, is_fixed = _GGA_FIX_MAP.get(fix, (NavSatStatus.STATUS_NO_FIX, False))
            fix_labels = {0:'No Fix',1:'3D Fix',2:'DGPS',4:'RTK Fixed',5:'RTK Float'}

            with self._state_lock:
                self._state.update({
                    'lat': lat, 'lon': lon, 'alt': alt,
                    'hdop': hdop, 'sv': sv,
                    'rtk_fixed': is_fixed,
                    'status_str': fix_labels.get(fix, f'Fix={fix}'),
                    '_nav_status': _fix_status,
                })
        except Exception:
            pass

    # ── Publish callback (10 Hz) ──────────────────────────
    def _publish_cb(self):
        with self._state_lock:
            s = dict(self._state)

        if not s['rtk_fixed'] or s['lat'] is None:
            return

        msg = NavSatFix()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'

        msg.status.status  = s.get('_nav_status', NavSatStatus.STATUS_GBAS_FIX)
        msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_GALILEO

        msg.latitude  = s['lat']
        msg.longitude = s['lon']
        msg.altitude  = s['alt']

        # diagonal covariance from HDOP (σ ≈ HDOP * 0.3 m baseline)
        sigma_h = s['hdop'] * 0.3
        sigma_v = sigma_h * 1.5
        msg.position_covariance = [
            sigma_h**2, 0.0,        0.0,
            0.0,        sigma_h**2, 0.0,
            0.0,        0.0,        sigma_v**2,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self._pub.publish(msg)

    # ── Status print (every 2 s) ──────────────────────────
    def _print_status(self):
        with self._state_lock:
            s = dict(self._state)

        lat_str = f"{s['lat']:.8f}" if s['lat'] is not None else '---'
        lon_str = f"{s['lon']:.8f}" if s['lon'] is not None else '---'
        alt_str = f"{s['alt']:.3f} m" if s['alt'] is not None else '---'

        if s['rtk_fixed']:
            pub_str = 'PUBLISHING'
        else:
            pub_str = f"NOT publishing (indoor / weak signal — {s['status_str']})"

        print(
            f"[RTK] {s['status_str']:12s}  "
            f"Lat: {lat_str}  Lon: {lon_str}  Alt: {alt_str}  "
            f"SV:{s['sv']}  HDOP:{s['hdop']:.1f}  "
            f"RTCM:{s['rtcm_count']}  → {pub_str}"
        )

    def destroy_node(self):
        self._radio.close()
        self._f9p.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RtkGpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()