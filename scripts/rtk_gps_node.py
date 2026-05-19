#!/usr/bin/env python3
"""
rtk_gps_node.py
  - Bridges RTCM corrections (with CRC validation) from radio → F9P
  - Parses NMEA GGA + UBX diagnostics (NAV-PVT, NAV-SIG)
  - Publishes sensor_msgs/NavSatFix on /gps/fix (RTK Fixed AND Float)
  - Publishes std_msgs/String JSON on /gps/rtk_status with full diagnostics
  - Displays: status, base station info, signal quality, firmware version
"""

import collections
import json
import threading
import struct
import time
import math

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

# ── Port config ────────────────────────────────────────────
RADIO_PORT = '/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0E6KL6-if00-port0'
F9P_PORT   = '/dev/ttyACM0'
BAUD_RADIO = 57600
BAUD_F9P   = 115200

PUBLISH_HZ      = 10
STATUS_INTERVAL = 2.0

# GNSS radio quality: sliding window (seconds)
GNSS_RADIO_WINDOW = 10.0


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

CRC24Q_POLY = 0x1864CFB

def _crc24q(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= CRC24Q_POLY
    return crc & 0xFFFFFF

def _parse_rtcm_1005(pkt: bytes) -> tuple:
    """Returns (station_id, lat, lon, alt) or None."""
    if len(pkt) < 22:
        return None
    data = pkt[3:-3]
    bits = ''.join(f'{b:08b}' for b in data)
    if len(bits) < 152:
        return None
    pos = 12
    sid = int(bits[pos:pos+12], 2); pos += 12
    pos += 6 + 4
    def s38(p):
        v = int(bits[p:p+38], 2)
        return v - (1 << 38) if v & (1 << 37) else v
    x = s38(pos) * 0.0001; pos += 38
    pos += 2
    y = s38(pos) * 0.0001; pos += 38
    pos += 2
    z = s38(pos) * 0.0001
    a = 6378137.0; f = 1.0 / 298.257223563
    b = a * (1 - f)
    e2 = (a*a - b*b) / (a*a)
    ep2 = (a*a - b*b) / (b*b)
    p = math.sqrt(x*x + y*y)
    th = math.atan2(z * a, p * b)
    lon = math.atan2(y, x)
    lat = math.atan2(z + ep2*b*math.sin(th)**3, p - e2*a*math.cos(th)**3)
    N = a / math.sqrt(1 - e2*math.sin(lat)**2)
    alt = p / math.cos(lat) - N
    return sid, math.degrees(lat), math.degrees(lon), alt

def _configure_f9p(ser: serial.Serial) -> str:
    """Configure F9P and return firmware version."""
    time.sleep(0.5)
    ser.reset_input_buffer()
    fw_ver = "unknown"
    ser.write(_ubx_packet(0x0A, 0x04))
    time.sleep(0.5)
    data = ser.read(2048)
    if b'\xb5\x62\x0a\x04' in data:
        i = data.find(b'\xb5\x62\x0a\x04')
        if len(data) >= i + 50:
            fw_ver = data[i+6:i+46].decode('ascii', errors='ignore').strip('\x00')
    ser.reset_input_buffer()
    cfg_keys = [
        (0x10770001, 1,   'B'),
        (0x10770002, 1,   'B'),
        (0x10770004, 1,   'B'),
        (0x10780002, 1,   'B'),
        (0x10730004, 1,   'B'),
        (0x10A3002E, 1,   'B'),
        (0x20030001, 0,   'B'),
        (0x20140011, 3,   'B'),
        (0x20110021, 0,   'B'),
        (0x30210001, 200, 'H'),
        (0x30210002, 1,   'H'),
        (0x1031001f, 1,   'B'),
        (0x10310025, 1,   'B'),
        (0x10310021, 1,   'B'),
    ]
    payload = struct.pack('<BBH', 0, 1, 0)
    for key, val, fmt in cfg_keys:
        payload += struct.pack('<I' + fmt, key, val)
    ser.write(_ubx_packet(0x06, 0x8A, payload))
    time.sleep(0.3)
    ack = ser.read(256)
    if b'\xb5\x62\x05\x01' in ack:
        print(f'[F9P] Config ACK. Firmware: {fw_ver}')
    else:
        print(f'[F9P] Config NAK. Firmware: {fw_ver}')
    ser.reset_input_buffer()
    return fw_ver


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

_GGA_FIX_MAP = {
    0: (NavSatStatus.STATUS_NO_FIX,   'No Fix',    False, False),
    1: (NavSatStatus.STATUS_FIX,      '3D Fix',    False, False),
    2: (NavSatStatus.STATUS_SBAS_FIX, 'DGPS',      False, False),
    4: (NavSatStatus.STATUS_GBAS_FIX, 'RTK Fixed', True,  True),
    5: (NavSatStatus.STATUS_GBAS_FIX, 'RTK Float', True,  False),
}


class RtkGpsNode(Node):
    def __init__(self):
        super().__init__('rtk_gps_node')

        self._pub        = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self._status_pub = self.create_publisher(String, '/gps/rtk_status', 10)
        self._timer        = self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)
        self._status_timer = self.create_timer(STATUS_INTERVAL, self._publish_status)
        self._print_timer  = self.create_timer(STATUS_INTERVAL, self._print_status)

        self._state_lock = threading.Lock()
        self._state = {
            'lat': None, 'lon': None, 'alt': None,
            'hdop': 99.9, 'sv': 0,
            'rtk_fixed': False, 'rtk_float': False, 'status_str': 'No Fix',
            'rtcm_count': 0, 'rtcm_bad': 0,
            'carrSoln': None, 'hAcc': None, 'vAcc': None,
            'strong': 0, 'cr_used': 0, 'l2_cr': 0,
            'base_id': None, 'base_lat': None, 'base_lon': None, 'base_alt': None,
            'fw_ver': 'unknown',
            '_nav_status': NavSatStatus.STATUS_NO_FIX,
        }

        # Sliding window for GNSS radio quality: (timestamp, is_bad)
        self._rtcm_window: collections.deque = collections.deque()
        self._rtcm_window_lock = threading.Lock()

        try:
            self._radio = serial.Serial(RADIO_PORT, BAUD_RADIO, timeout=0.1)
            self._f9p   = serial.Serial(F9P_PORT,   BAUD_F9P,   timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open serial port: {e}')
            raise

        self._state['fw_ver'] = _configure_f9p(self._f9p)
        self.get_logger().info('F9P configured. Starting threads...')

        threading.Thread(target=self._bridge_thread, daemon=True).start()
        threading.Thread(target=self._f9p_thread,    daemon=True).start()

    def _gnss_radio_quality(self) -> int:
        """Return 0-100 score for GPS base <-> GNSS radio link from RTCM receive stats."""
        now = time.time()
        cutoff = now - GNSS_RADIO_WINDOW
        with self._rtcm_window_lock:
            # Evict old entries
            while self._rtcm_window and self._rtcm_window[0][0] < cutoff:
                self._rtcm_window.popleft()
            total = len(self._rtcm_window)
            bad   = sum(1 for _, is_bad in self._rtcm_window if is_bad)

        if total == 0:
            return 0
        rate = total / GNSS_RADIO_WINDOW  # packets/sec over window
        bad_ratio = bad / total if total > 0 else 0.0
        # Nominal rate ~1-4 Hz; score saturates at 1.5 Hz
        rate_score = min(1.0, rate / 1.5)  # 1.5 Hz saturates to 100% in 10s window
        quality_factor = max(0.0, 1.0 - bad_ratio * 3.0)
        return int(rate_score * quality_factor * 100)

    # ── RTCM bridge: radio → F9P (with CRC validation) ────
    def _bridge_thread(self):
        rtcm_buf = b''
        while rclpy.ok():
            data = self._radio.read(4096)
            if not data:
                continue
            rtcm_buf += data

            while True:
                i = rtcm_buf.find(b'\xd3')
                if i < 0:
                    rtcm_buf = b''
                    break
                if i > 0:
                    rtcm_buf = rtcm_buf[i:]
                if len(rtcm_buf) < 3:
                    break
                plen = ((rtcm_buf[1] & 0x03) << 8) | rtcm_buf[2]
                total = plen + 6
                if total > 1029:
                    rtcm_buf = rtcm_buf[1:]
                    continue
                if len(rtcm_buf) < total:
                    break

                pkt = rtcm_buf[:total]
                calc = _crc24q(pkt[:-3])
                rx = (pkt[-3] << 16) | (pkt[-2] << 8) | pkt[-1]

                if calc != rx:
                    with self._state_lock:
                        self._state['rtcm_bad'] += 1
                    with self._rtcm_window_lock:
                        self._rtcm_window.append((time.time(), True))
                    rtcm_buf = rtcm_buf[1:]
                    continue

                mt = ((pkt[3] << 4) | (pkt[4] >> 4)) & 0xFFF
                if mt == 1005:
                    r = _parse_rtcm_1005(pkt)
                    if r:
                        with self._state_lock:
                            self._state['base_id'], self._state['base_lat'], \
                            self._state['base_lon'], self._state['base_alt'] = r

                self._f9p.write(pkt)
                with self._state_lock:
                    self._state['rtcm_count'] += 1
                with self._rtcm_window_lock:
                    self._rtcm_window.append((time.time(), False))
                rtcm_buf = rtcm_buf[total:]

    # ── F9P reader: NMEA + UBX ────────────────────────────
    def _f9p_thread(self):
        buf = b''
        last_poll = [0.0]
        while rclpy.ok():
            chunk = self._f9p.read(256)
            if chunk:
                buf += chunk

            while buf:
                i_n = buf.find(b'$')
                i_u = buf.find(b'\xb5\x62')
                if i_n < 0 and i_u < 0:
                    buf = b''
                    break
                if i_n < 0:
                    start, mode = i_u, 'ubx'
                elif i_u < 0:
                    start, mode = i_n, 'nmea'
                elif i_n < i_u:
                    start, mode = i_n, 'nmea'
                else:
                    start, mode = i_u, 'ubx'
                if start > 0:
                    buf = buf[start:]

                if mode == 'nmea':
                    j = buf.find(b'\n')
                    if j < 0:
                        break
                    line = buf[:j].decode('ascii', errors='ignore').strip()
                    buf = buf[j+1:]
                    self._parse_gga(line)
                else:
                    if len(buf) < 8:
                        break
                    length = struct.unpack_from('<H', buf, 4)[0]
                    if length > 2048:
                        buf = buf[2:]
                        continue
                    total = 8 + length
                    if len(buf) < total:
                        break
                    cls, mid = buf[2], buf[3]
                    payload = buf[6:6+length]
                    buf = buf[total:]
                    self._handle_ubx(cls, mid, payload)

            now = time.time()
            if now - last_poll[0] >= 2.0:
                try:
                    self._f9p.write(_ubx_packet(0x01, 0x07))
                    self._f9p.write(_ubx_packet(0x01, 0x43))
                    last_poll[0] = now
                except Exception:
                    pass

    def _parse_gga(self, line: str):
        try:
            if not line.startswith('$'):
                return
            parts = line.split(',')
            if parts[0] not in ('$GNGGA', '$GPGGA') or len(parts) <= 9:
                return
            lat = _nmea_to_decimal(parts[2], parts[3])
            lon = _nmea_to_decimal(parts[4], parts[5])
            fix = int(parts[6]) if parts[6] else 0
            sv = int(parts[7]) if parts[7] else 0
            hdop = float(parts[8]) if parts[8] else 99.9
            alt = float(parts[9]) if parts[9] else None
            if lat is None or lon is None or alt is None:
                return
            _fix_status, status_str, has_rtk, is_fixed = _GGA_FIX_MAP.get(
                fix, (NavSatStatus.STATUS_NO_FIX, f'Fix={fix}', False, False))
            with self._state_lock:
                self._state.update({
                    'lat': lat, 'lon': lon, 'alt': alt,
                    'hdop': hdop, 'sv': sv,
                    'rtk_fixed': is_fixed,
                    'rtk_float': has_rtk and not is_fixed,
                    'status_str': status_str,
                    '_nav_status': _fix_status,
                })
        except Exception:
            pass

    def _handle_ubx(self, cls: int, mid: int, payload: bytes):
        if cls == 0x01 and mid == 0x07 and len(payload) >= 92:  # NAV-PVT
            flags = payload[21]
            with self._state_lock:
                self._state['carrSoln'] = (flags >> 6) & 0x03
                self._state['hAcc'] = struct.unpack_from('<I', payload, 40)[0] / 1000.0
                self._state['vAcc'] = struct.unpack_from('<I', payload, 44)[0] / 1000.0

        elif cls == 0x01 and mid == 0x43 and len(payload) >= 8:  # NAV-SIG
            num = payload[5]
            strong = cr_used = l2_cr = 0
            L2_SIGS = {('GPS',3),('GPS',4),('GAL',5),('GAL',6),('BDS',2),('BDS',3),('GLO',2),('QZS',4),('QZS',5)}
            GNSS_MAP = {0:'GPS',1:'SBA',2:'GAL',3:'BDS',5:'QZS',6:'GLO'}
            for i in range(num):
                off = 8 + i * 16
                if off + 16 > len(payload):
                    break
                gnss = GNSS_MAP.get(payload[off], '?')
                sigId = payload[off + 2]
                cno = payload[off + 6]
                sigFlags = struct.unpack_from('<H', payload, off + 10)[0]
                cr_used_flag = bool(sigFlags & (1 << 4))
                if cno >= 40:
                    strong += 1
                if cr_used_flag:
                    cr_used += 1
                if (gnss, sigId) in L2_SIGS and cno >= 35 and cr_used_flag:
                    l2_cr += 1
            with self._state_lock:
                self._state['strong'] = strong
                self._state['cr_used'] = cr_used
                self._state['l2_cr'] = l2_cr

    # ── Publish NavSatFix (RTK Fixed AND Float) ───────────
    def _publish_cb(self):
        with self._state_lock:
            s = dict(self._state)
        if s['lat'] is None:
            return
        # Publish for RTK Fixed or Float (not bare 3D fix or no-fix)
        if not (s['rtk_fixed'] or s['rtk_float']):
            return
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status.status = s.get('_nav_status', NavSatStatus.STATUS_GBAS_FIX)
        msg.status.service = (NavSatStatus.SERVICE_GPS |
                              NavSatStatus.SERVICE_GLONASS |
                              NavSatStatus.SERVICE_GALILEO)
        msg.latitude  = s['lat']
        msg.longitude = s['lon']
        msg.altitude  = s['alt']
        # Float: larger covariance; Fixed: tighter
        scale = 1.0 if s['rtk_fixed'] else 3.0
        sigma_h = s['hdop'] * 0.3 * scale
        sigma_v = sigma_h * 1.5
        msg.position_covariance = [
            sigma_h**2, 0.0, 0.0,
            0.0, sigma_h**2, 0.0,
            0.0, 0.0, sigma_v**2,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._pub.publish(msg)

    # ── Publish full RTK status JSON ──────────────────────
    def _publish_status(self):
        with self._state_lock:
            s = dict(self._state)
        gnss_quality = self._gnss_radio_quality()

        def _r(v, dp): return round(v, dp) if v is not None else None

        carr_map = {0: 'None', 1: 'Float', 2: 'Fixed', None: 'Unknown'}
        status = {
            'rtk_mode':      s['status_str'],
            'rtk_fixed':     s['rtk_fixed'],
            'rtk_float':     s['rtk_float'],
            'lat':           _r(s['lat'],      6),
            'lon':           _r(s['lon'],      6),
            'alt':           _r(s['alt'],      2),
            'hdop':          _r(s['hdop'],     1),
            'sv':            s['sv'],
            'h_acc':         _r(s['hAcc'],     3),
            'v_acc':         _r(s['vAcc'],     3),
            'carr_soln':     carr_map.get(s['carrSoln'], 'Unknown'),
            'strong_sv':     s['strong'],
            'cr_used':       s['cr_used'],
            'l2_cr':         s['l2_cr'],
            'rtcm_count':    s['rtcm_count'],
            'rtcm_bad':      s['rtcm_bad'],
            'gnss_radio_quality': gnss_quality,
            'base_id':       s['base_id'],
            'base_lat':      _r(s['base_lat'], 6),
            'base_lon':      _r(s['base_lon'], 6),
            'base_alt':      _r(s['base_alt'], 2),
            'fw_ver':        s['fw_ver'],
        }
        # Baseline distance (rough metres)
        if s['lat'] is not None and s['base_lat'] is not None:
            status['baseline_m'] = round(
                math.sqrt((s['lat'] - s['base_lat'])**2 +
                          (s['lon'] - s['base_lon'])**2) * 111000, 1)
        else:
            status['baseline_m'] = None

        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    # ── Console status print ──────────────────────────────
    def _print_status(self):
        with self._state_lock:
            s = dict(self._state)
        gnss_quality = self._gnss_radio_quality()

        lat_str = f"{s['lat']:.8f}" if s['lat'] is not None else '---'
        lon_str = f"{s['lon']:.8f}" if s['lon'] is not None else '---'
        alt_str = f"{s['alt']:.2f} m" if s['alt'] is not None else '---'
        carr_map = {0:'None', 1:'Float', 2:'FIXED', None:'---'}
        quality = (f"carrSoln:{carr_map.get(s['carrSoln'], '---'):6s}  "
                   f"hAcc:{s['hAcc'] if s['hAcc'] else '-':<6}  "
                   f"strong:{s['strong']}  cr-used:{s['cr_used']}  L2-cr:{s['l2_cr']}")
        if s['base_lat'] is not None:
            baseline = math.sqrt((s['lat'] - s['base_lat'])**2 +
                                 (s['lon'] - s['base_lon'])**2) * 111000
            base_str = (f"id:{s['base_id']}  {s['base_lat']:.6f},{s['base_lon']:.6f}  "
                        f"alt:{s['base_alt']:.1f}m  baseline:{baseline:.1f}m")
        else:
            base_str = "waiting for 1005..."

        status = s['status_str']
        if s["rtk_fixed"]:
            color  = "\033[38;2;180;245;255m"
            suffix = "  ✓ FIXED PUBLISHING"
        elif s["rtk_float"]:
            color  = "\033[38;2;150;200;230m"
            suffix = "  ~ FLOAT PUBLISHING"
        else:
            color  = "\033[38;2;150;150;150m"
            suffix = ""

        status_style = "\033[1m"
        if s["rtk_fixed"]:
            status_style += "\033[4m"  # underline for fixed

        status_fmt = f"{status_style}{status:12s}\033[22m\033[24m"

        line = (
            f"{'='*10}\n  {status_fmt}  SV:{s['sv']}"
            f"\n[Base]  {base_str}"
            f"\nHDOP:{s['hdop']:.1f}  RTCM:{s['rtcm_count']} "
            f"\n[Quality]  {quality}"
            f"\nLat:{lat_str}  Lon:{lon_str}  Alt:{alt_str}  "
            f"\n[FW:{s['fw_ver']:<10s}]  "
            f"\n(bad:{s['rtcm_bad']})  GNSS-Radio:{gnss_quality}%\n"
        )

        print(f"{color}{line}{suffix}\033[0m")

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