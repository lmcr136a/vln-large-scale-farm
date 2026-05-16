import serial
import threading
import struct
import time
import math

FACET_PORT = '/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0E6KL6-if00-port0'
F9P_PORT   = '/dev/ttyACM0'  # u-blox CDC, stable as long as no other ACM device
BAUD_FACET = 57600
BAUD_F9P   = 115200

facet = serial.Serial(FACET_PORT, BAUD_FACET, timeout=0.1)
f9p   = serial.Serial(F9P_PORT,   BAUD_F9P,   timeout=0.2)

def ubx_checksum(msg):
    ck_a = ck_b = 0
    for b in msg:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])

def ubx_packet(cls, mid, payload=b''):
    msg = bytes([cls, mid]) + struct.pack('<H', len(payload)) + payload
    return b'\xb5\x62' + msg + ubx_checksum(msg)

def read_ack(cls_id, msg_id, timeout=1.5):
    """Wait for UBX-ACK-ACK/NAK matching (cls_id, msg_id).
    Returns True (ACK), False (NAK), or None (timeout)."""
    deadline = time.time() + timeout
    buf = b''
    while time.time() < deadline:
        buf += f9p.read(256)
        while True:
            i = buf.find(b'\xb5\x62\x05')
            if i < 0:
                buf = buf[-2:]
                break
            if len(buf) < i + 10:
                buf = buf[i:]
                break
            ack_type = buf[i + 3]   # 0x01 = ACK, 0x00 = NAK
            ack_cls  = buf[i + 6]
            ack_mid  = buf[i + 7]
            buf = buf[i + 10:]
            if ack_cls == cls_id and ack_mid == msg_id:
                return ack_type == 0x01
    return None

def valget(key, fmt='B', timeout=1.5):
    """Read back a single CFG key from RAM. Returns value or None."""
    f9p.reset_input_buffer()
    payload = struct.pack('<BBH', 0, 0, 0) + struct.pack('<I', key)
    f9p.write(ubx_packet(0x06, 0x8B, payload))
    deadline = time.time() + timeout
    buf = b''
    while time.time() < deadline:
        buf += f9p.read(256)
        i = buf.find(b'\xb5\x62\x06\x8b')
        if i >= 0 and len(buf) >= i + 8:
            length = struct.unpack_from('<H', buf, i + 4)[0]
            if len(buf) >= i + 6 + length + 2:
                pl = buf[i + 6 : i + 6 + length]
                if len(pl) >= 9:
                    val_bytes = pl[8:]
                    sz = struct.calcsize('<' + fmt)
                    if len(val_bytes) >= sz:
                        return struct.unpack_from('<' + fmt, val_bytes)[0]
                return None
    return None

def configure_f9p():
    """F9P: enable UART1 RTCM3 input + all GNSS constellations,
    verify with ACK and read-back via VALGET."""
    time.sleep(0.3)
    f9p.reset_input_buffer()

    cfg_keys = [
        (0x10730004, 1,   'B'),   # CFG-UART1INPROT-RTCM3X
        (0x10730001, 1,   'B'),   # CFG-UART1INPROT-UBX
        (0x10740002, 1,   'B'),   # CFG-UART1OUTPROT-NMEA
        (0x1031001f, 1,   'B'),   # CFG-SIGNAL-GPS_ENA
        (0x10310001, 1,   'B'),   # CFG-SIGNAL-GPS_L1CA_ENA
        (0x10310003, 1,   'B'),   # CFG-SIGNAL-GPS_L2C_ENA
        (0x10310021, 1,   'B'),   # CFG-SIGNAL-GAL_ENA
        (0x10310022, 1,   'B'),   # CFG-SIGNAL-BDS_ENA
        (0x10310025, 1,   'B'),   # CFG-SIGNAL-GLO_ENA
        # *** CRITICAL: 2=Float-only, 3=allow Fixed (default 3, but verify)
        (0x20140011, 3,   'B'),   # CFG-NAVHPG-DGNSSMODE
        # 0=portable, 1=stationary, 4=automotive, 8=bike
        (0x20110021, 0,   'B'),   # CFG-NAVSPG-DYNMODEL
        # Measurement rate (ms) and nav rate (cycles)
        (0x30210001, 200, 'H'),   # CFG-RATE-MEAS = 200 ms (5 Hz)
        (0x30210002, 1,   'H'),   # CFG-RATE-NAV  = every measurement
    ]
    payload = struct.pack('<BBH', 0, 1, 0)
    for key, val, fmt in cfg_keys:
        payload += struct.pack('<I' + fmt, key, val)
    f9p.write(ubx_packet(0x06, 0x8A, payload))

    ack = read_ack(0x06, 0x8A)
    if ack is True:
        print("[F9P] CFG-VALSET  -> ACK")
    elif ack is False:
        print("[F9P] CFG-VALSET  -> NAK  (firmware rejected one or more keys)")
    else:
        print("[F9P] CFG-VALSET  -> TIMEOUT  (no response from F9P)")

    checks = [
        (0x10730004, 'UART1INPROT-RTCM3X', 'B', 1),
        (0x10730001, 'UART1INPROT-UBX',    'B', 1),
        (0x1031001f, 'SIGNAL-GPS_ENA',     'B', 1),
        (0x10310021, 'SIGNAL-GAL_ENA',     'B', 1),
        (0x10310022, 'SIGNAL-BDS_ENA',     'B', 1),
        (0x10310025, 'SIGNAL-GLO_ENA',     'B', 1),
        (0x20140011, 'NAVHPG-DGNSSMODE',   'B', 3),   # MUST be 3
        (0x20110021, 'NAVSPG-DYNMODEL',    'B', 0),
        (0x30210001, 'RATE-MEAS (ms)',     'H', 200),
        (0x30210002, 'RATE-NAV (cycles)',  'H', 1),
    ]
    print("[F9P] Verifying config via CFG-VALGET (RAM layer):")
    for key, name, fmt, expected in checks:
        v = valget(key, fmt)
        mark = 'OK' if v == expected else f'!! expected {expected}'
        print(f"        {name:<22} = {v}  [{mark}]")

configure_f9p()

RTK_STATUS = {
    0: 'No Fix', 1: 'Dead Reckoning', 2: '2D Fix',
    3: '3D Fix',  4: 'GNSS+DR',       5: 'RTK Float', 6: 'RTK Fixed'
}

# RTCM3 message types that actually drive RTK convergence
RTK_CORRECTION_TYPES = (
    set(range(1001, 1005)) | set(range(1009, 1013))  # legacy GPS/GLO
    | {1074, 1084, 1094, 1114, 1124}                 # MSM4
    | {1077, 1087, 1097, 1117, 1127}                 # MSM7
    | {1005, 1006, 1007, 1033, 1230}                 # station info / GLO biases
)

def classify_rtcm(mt):
    if mt in RTK_CORRECTION_TYPES:
        return 'rtk'
    if 4070 <= mt <= 4095:
        return 'prop'  # u-blox / vendor proprietary
    return 'other'

state = {
    'lat': None, 'lon': None, 'alt': None,
    'fix': 0, 'status': 'No Fix', 'sv': 0,
    'rtcm': 0, 'rtcm_bytes': 0,
    'rtcm_types': {},
    'rtk_corr_count': 0,
    'last_rtcm_time': 0.0,
    'rtcm_bad_crc': 0,
    'rtcm_resync': 0,
    'base_lat': None, 'base_lon': None, 'base_alt': None, 'base_id': None,
    # UBX diagnostic fields
    'carrSoln': None,   # 0=none, 1=Float, 2=Fixed
    'hAcc_m':   None,   # horizontal accuracy estimate (m)
    'vAcc_m':   None,   # vertical accuracy estimate (m)
    'pDOP':     None,
    'n_l2':     0,      # # signals tracked on L2/E5b/B2
    'n_l2_cr':  0,      # # of those with C/N0>=35 and carrier-range used
    'n_strong': 0,      # # of all signals with C/N0>=40
    'n_cr':     0,      # # signals contributing carrier-range
    'n_rtcm':   0,      # # signals with corrSource==RTCM3-OSR (4)
}
DISPLAY_LINES = 9

CRC24Q_POLY = 0x1864CFB

def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= CRC24Q_POLY
    return crc & 0xFFFFFF

def parse_rtcm_1005(pkt):
    """Returns (station_id, ecef_x_m, ecef_y_m, ecef_z_m) or None."""
    if len(pkt) < 22:
        return None
    data = pkt[3:-3]
    bits = ''.join(f'{b:08b}' for b in data)
    if len(bits) < 152:
        return None
    pos = 12  # skip message type
    sid = int(bits[pos:pos+12], 2);                pos += 12
    pos += 6 + 4  # ITRF year + 4 indicator bits
    def s38(p):
        v = int(bits[p:p+38], 2)
        return v - (1 << 38) if v & (1 << 37) else v
    x = s38(pos) * 0.0001; pos += 38
    pos += 2
    y = s38(pos) * 0.0001; pos += 38
    pos += 2
    z = s38(pos) * 0.0001
    return sid, x, y, z

def ecef_to_llh(x, y, z):
    a  = 6378137.0
    f  = 1.0 / 298.257223563
    b  = a * (1 - f)
    e2 = (a*a - b*b) / (a*a)
    ep2 = (a*a - b*b) / (b*b)
    p = math.sqrt(x*x + y*y)
    th = math.atan2(z * a, p * b)
    lon = math.atan2(y, x)
    lat = math.atan2(z + ep2*b*math.sin(th)**3,
                     p - e2*a*math.cos(th)**3)
    N = a / math.sqrt(1 - e2*math.sin(lat)**2)
    alt = p / math.cos(lat) - N
    return math.degrees(lat), math.degrees(lon), alt

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2 * R * math.asin(math.sqrt(a))
RECORD_INTERVAL = 0.2
trajectory   = []
_last_record = [0.0]

def maybe_record():
    now = time.time()
    if (now - _last_record[0] >= RECORD_INTERVAL
            and state['lat'] is not None
            and state['fix'] == 6):
        trajectory.append((state['lat'], state['lon'], state['alt'], now, state['fix']))
        _last_record[0] = now

_last_display = [0.0]
DISPLAY_INTERVAL = 0.2

def display():
    now = time.time()
    if now - _last_display[0] < DISPLAY_INTERVAL:
        return
    _last_display[0] = now
    lat = f"{state['lat']:.8f}"   if state['lat'] is not None else '---'
    lon = f"{state['lon']:.8f}"   if state['lon'] is not None else '---'
    alt = f"{state['alt']:.3f} m" if state['alt'] is not None else '---'

    # Build "type:count" string sorted by count desc, marking category
    items = sorted(state['rtcm_types'].items(), key=lambda kv: -kv[1])
    parts = []
    for mt, c in items[:8]:
        tag = {'rtk': '*', 'prop': 'p', 'other': '?'}[classify_rtcm(mt)]
        parts.append(f"{mt}{tag}:{c}")
    types_str = ' '.join(parts) or '---'

    age = time.time() - state['last_rtcm_time'] if state['last_rtcm_time'] else -1
    age_str = f"{age:.1f}s ago" if age >= 0 else 'never'

    # Convergence diagnosis
    valid_types = set(state['rtcm_types'].keys())
    have_station = bool(valid_types & {1005, 1006})
    have_msm     = bool(valid_types & {1074, 1077, 1084, 1087, 1094, 1097, 1124, 1127})

    if state['rtcm'] == 0 and state['rtcm_resync'] > 100:
        diag = f"All bytes failed framing ({state['rtcm_resync']} discarded) - wrong port/baud/format"
    elif state['rtcm_bad_crc'] > state['rtcm'] and state['rtcm_bad_crc'] > 5:
        diag = f"Mostly CRC-fail garbage (bad:{state['rtcm_bad_crc']} ok:{state['rtcm']}) - link corruption or wrong baud"
    elif state['rtk_corr_count'] == 0 and state['rtcm'] > 0:
        diag = "Stream OK but NO standard RTK msgs (no 1005/107x/108x/109x/112x) - reconfigure Facet base"
    elif not have_station and have_msm:
        diag = "MSM present but no base coords (1005/1006) - Facet not in fixed/survey Base mode"
    elif have_station and not have_msm:
        diag = "Base coords present but no MSM observations - enable MSM4/7 on Facet"
    elif state['fix'] == 6:
        diag = "RTK FIXED"
    elif state['fix'] == 5:
        diag = "RTK Float - converging..."
    else:
        diag = f"corrections OK ({state['rtk_corr_count']}) but no RTK fix yet"

    n = len(trajectory)

    if state['base_lat'] is not None and state['lat'] is not None:
        baseline = haversine_m(state['lat'], state['lon'],
                               state['base_lat'], state['base_lon'])
        base_info = (f"id:{state['base_id']}  "
                     f"{state['base_lat']:.6f},{state['base_lon']:.6f}  "
                     f"alt:{state['base_alt']:.1f}m  baseline:{baseline/1000:.2f}km")
    elif state['base_lat'] is not None:
        base_info = (f"id:{state['base_id']}  "
                     f"{state['base_lat']:.6f},{state['base_lon']:.6f}  "
                     f"alt:{state['base_alt']:.1f}m  (rover pos unknown)")
    else:
        base_info = "waiting for 1005..."

    cs_map = {0:'None', 1:'Float', 2:'FIXED', None:'---'}
    if state['hAcc_m'] is not None:
        quality = (f"carrSoln:{cs_map[state['carrSoln']]:<6} "
                   f"hAcc:{state['hAcc_m']:.3f}m  vAcc:{state['vAcc_m']:.3f}m  "
                   f"pDOP:{state['pDOP']:.2f}  "
                   f"L2:{state['n_l2']}/L2-cr:{state['n_l2_cr']}  "
                   f"strong:{state['n_strong']}  cr-used:{state['n_cr']}  "
                   f"rtcm-applied:{state['n_rtcm']}")
    else:
        quality = "waiting for UBX poll response..."

    print('\033[H\033[J', end='')   # clear screen + cursor home
    print(f"  [F9P Rover]   Lat: {lat:<16} Lon: {lon:<18} Alt: {alt}")
    print(f"  [RTK Status]  {state['status']:<30} SV:{state['sv']}")
    print(f"  [RTCM]        ok:{state['rtcm']}  rtk:{state['rtk_corr_count']}  "
          f"bad-crc:{state['rtcm_bad_crc']}  resync:{state['rtcm_resync']}B  last:{age_str}")
    print(f"  [RTCM types]  {types_str}")
    print(f"  [Base]        {base_info}")
    print(f"  [Quality]     {quality}")
    print(f"  [Diagnosis]   {diag}")
    print(f"  [Recording]   {n} pts  (every {RECORD_INTERVAL}s, RTK Fixed only)")

# ── Facet bridge: RTCM -> F9P (with CRC24Q validation) ───
def bridge():
    rtcm_buf = b''
    while True:
        data = facet.read(4096)
        if not data:
            continue
        rtcm_buf += data

        while True:
            # Find next sync byte
            i = rtcm_buf.find(b'\xd3')
            if i < 0:
                state['rtcm_resync'] += len(rtcm_buf)
                rtcm_buf = b''
                break
            if i > 0:
                state['rtcm_resync'] += i
                rtcm_buf = rtcm_buf[i:]

            if len(rtcm_buf) < 3:
                break
            plen  = ((rtcm_buf[1] & 0x03) << 8) | rtcm_buf[2]
            total = plen + 6
            if total > 1029:  # RTCM3 max frame size
                state['rtcm_resync'] += 1
                rtcm_buf = rtcm_buf[1:]
                continue
            if len(rtcm_buf) < total:
                break

            pkt = rtcm_buf[:total]
            calc = crc24q(pkt[:-3])
            rx   = (pkt[-3] << 16) | (pkt[-2] << 8) | pkt[-1]
            if calc != rx:
                state['rtcm_bad_crc'] += 1
                state['rtcm_resync'] += 1
                rtcm_buf = rtcm_buf[1:]
                continue

            mt = ((pkt[3] << 4) | (pkt[4] >> 4)) & 0xFFF
            state['rtcm_types'][mt] = state['rtcm_types'].get(mt, 0) + 1
            if classify_rtcm(mt) == 'rtk':
                state['rtk_corr_count'] += 1
            if mt == 1005 and state['base_lat'] is None:
                r = parse_rtcm_1005(pkt)
                if r:
                    sid, ex, ey, ez = r
                    blat, blon, balt = ecef_to_llh(ex, ey, ez)
                    state['base_id']  = sid
                    state['base_lat'] = blat
                    state['base_lon'] = blon
                    state['base_alt'] = balt
            state['rtcm'] += 1
            state['rtcm_bytes'] += total
            state['last_rtcm_time'] = time.time()
            f9p.write(pkt)
            rtcm_buf = rtcm_buf[total:]

threading.Thread(target=bridge, daemon=True).start()

# ── F9P NMEA 파싱 ──────────────────────────────────────────
def nmea_to_decimal(value, direction):
    try:
        dot = value.index('.')
        degrees = float(value[:dot-2])
        minutes = float(value[dot-2:])
        decimal = degrees + minutes / 60.0
        if direction in ('S', 'W'):
            decimal = -decimal
        return decimal
    except:
        return None

def parse_nmea(line):
    try:
        if not line.startswith('$'): return
        parts = line.split(',')
        tag = parts[0]
        if tag in ('$GNGGA', '$GPGGA') and len(parts) > 9:
            lat = nmea_to_decimal(parts[2], parts[3])
            lon = nmea_to_decimal(parts[4], parts[5])
            fix = int(parts[6]) if parts[6] else 0
            sv  = int(parts[7]) if parts[7] else 0
            alt = float(parts[9]) if parts[9] else None
            fix_map = {0:'No Fix',1:'3D Fix',2:'DGPS',4:'RTK Fixed',5:'RTK Float'}
            fix_rtk = {0:0, 1:3, 2:2, 4:6, 5:5}
            if lat and lon:
                state['lat'] = lat
                state['lon'] = lon
                state['fix'] = fix_rtk.get(fix, 0)
                state['status'] = fix_map.get(fix, f'Fix={fix}')
                state['sv']  = sv
                if alt is not None:
                    state['alt'] = alt
    except:
        pass

print("Monitoring F9P rover (/dev/ttyACM0)...  Ctrl+C -> stop & 3D visualize")
for _ in range(DISPLAY_LINES): print()

# ── UBX parsing & periodic polling ────────────────────────
GNSS_NAMES = {0:'GPS', 1:'SBA', 2:'GAL', 3:'BDS', 5:'QZS', 6:'GLO'}
L2_SIGS = {
    ('GPS', 3), ('GPS', 4),    # L2CL, L2CM
    ('GAL', 5), ('GAL', 6),    # E5bI, E5bQ
    ('BDS', 2), ('BDS', 3),    # B2I
    ('GLO', 2),                # L2OF
    ('QZS', 4), ('QZS', 5),    # L2CM, L2CL
}

def parse_nav_pvt(p):
    if len(p) < 92: return
    flags = p[21]
    state['carrSoln'] = (flags >> 6) & 0x03
    state['hAcc_m']   = struct.unpack_from('<I', p, 40)[0] / 1000.0
    state['vAcc_m']   = struct.unpack_from('<I', p, 44)[0] / 1000.0
    state['pDOP']     = struct.unpack_from('<H', p, 76)[0] * 0.01

def parse_nav_sig(p):
    if len(p) < 8: return
    num = p[5]
    n_l2 = n_l2_cr = n_strong = n_cr = n_rtcm = 0
    for i in range(num):
        off = 8 + i * 16
        if off + 16 > len(p): break
        gnss     = GNSS_NAMES.get(p[off], '?')
        sigId    = p[off + 2]
        cno      = p[off + 6]            # offset 6 (not 4)
        corrSrc  = p[off + 8]            # 0=none 1=SBAS 2=BDS 4=RTCM3-OSR 5=SSR 7=SPARTN
        sigFlags = struct.unpack_from('<H', p, off + 10)[0]   # offset 10 (not 8)
        pr_used = bool(sigFlags & (1 << 3))
        cr_used = bool(sigFlags & (1 << 4))
        if cno >= 40:    n_strong += 1
        if cr_used:      n_cr += 1
        if corrSrc == 4: n_rtcm += 1
        if (gnss, sigId) in L2_SIGS:
            n_l2 += 1
            if cno >= 35 and cr_used:
                n_l2_cr += 1
    state['n_l2']     = n_l2
    state['n_l2_cr']  = n_l2_cr
    state['n_strong'] = n_strong
    state['n_cr']     = n_cr
    state['n_rtcm']   = n_rtcm

_last_poll = [0.0]
POLL_INTERVAL = 2.0
def maybe_poll_ubx():
    now = time.time()
    if now - _last_poll[0] >= POLL_INTERVAL:
        try:
            f9p.write(ubx_packet(0x01, 0x07))   # NAV-PVT
            f9p.write(ubx_packet(0x01, 0x43))   # NAV-SIG
        except Exception:
            pass
        _last_poll[0] = now

def handle_ubx(cls, mid, payload):
    if   cls == 0x01 and mid == 0x07: parse_nav_pvt(payload)
    elif cls == 0x01 and mid == 0x43: parse_nav_sig(payload)

# ── Mixed NMEA + UBX read loop ────────────────────────────
f9p_buf = b''

def drain_f9p_buf():
    """Parse all complete NMEA lines and UBX frames from f9p_buf."""
    global f9p_buf
    while f9p_buf:
        i_n = f9p_buf.find(b'$')
        i_u = f9p_buf.find(b'\xb5\x62')
        if i_n < 0 and i_u < 0:
            f9p_buf = b''
            return
        if i_n < 0:                    start, mode = i_u, 'ubx'
        elif i_u < 0:                  start, mode = i_n, 'nmea'
        elif i_n < i_u:                start, mode = i_n, 'nmea'
        else:                          start, mode = i_u, 'ubx'
        if start > 0:
            f9p_buf = f9p_buf[start:]
        if mode == 'nmea':
            j = f9p_buf.find(b'\n')
            if j < 0: return
            line = f9p_buf[:j].decode('ascii', errors='ignore').strip()
            f9p_buf = f9p_buf[j+1:]
            parse_nmea(line)
        else:
            if len(f9p_buf) < 8: return
            length = struct.unpack_from('<H', f9p_buf, 4)[0]
            if length > 2048:
                f9p_buf = f9p_buf[2:]   # garbage, skip sync
                continue
            total = 8 + length
            if len(f9p_buf) < total: return
            cls, mid = f9p_buf[2], f9p_buf[3]
            payload  = f9p_buf[6:6+length]
            f9p_buf  = f9p_buf[total:]
            handle_ubx(cls, mid, payload)

try:
    while True:
        chunk = f9p.read(256)
        if chunk:
            f9p_buf += chunk
            drain_f9p_buf()
        maybe_poll_ubx()
        maybe_record()
        display()

except KeyboardInterrupt:
    pass
finally:
    facet.close()
    f9p.close()


# ── 종료 후 3D 시각화 ──────────────────────────────────────
print(f"\nRecorded {len(trajectory)} points. Generating 3D plot...")

if len(trajectory) < 2:
    print("데이터 부족 (최소 2점 필요). 더 오래 실행 후 종료해줘.")
else:
    import numpy as np
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from matplotlib.collections import LineCollection
    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    lats  = np.array([p[0] for p in trajectory])
    lons  = np.array([p[1] for p in trajectory])
    alts  = np.array([p[2] for p in trajectory])
    times = np.array([p[3] for p in trajectory])
    fixes = np.array([p[4] for p in trajectory])

    # lat/lon → ENU 미터 (원점: 시작점)
    lat0, lon0, alt0 = lats[0], lons[0], alts[0]
    R = 6378137.0
    x = (lons - lon0) * (math.pi/180) * R * math.cos(math.radians(lat0))
    y = (lats - lat0) * (math.pi/180) * R
    z = alts - alt0

    t_norm = (times - times[0]) / max(times[-1] - times[0], 1e-9)

    duration  = times[-1] - times[0]
    fix_names = {3:'3D Fix', 4:'GNSS+DR', 5:'RTK Float', 6:'RTK Fixed'}
    uniq_fix  = ', '.join(sorted({fix_names.get(int(f), str(f)) for f in fixes}))

    pts   = np.array([x, y, z]).T
    segs  = [[pts[k], pts[k+1]] for k in range(len(pts)-1)]
    t_mid = (t_norm[:-1] + t_norm[1:]) / 2
    sz    = np.where(fixes == 6, 40, np.where(fixes == 5, 25, 12))
    pad   = max(x.ptp(), y.ptp(), z.ptp()) * 0.05 + 0.1
    z_floor = z.min() - 0.5

    fig = plt.figure(figsize=(18, 9))
    fig.suptitle(
        f"MTi Rover Trajectory  ·  {len(trajectory)} pts  ·  {duration:.1f} s  ·  Fix: {uniq_fix}\n"
        f"Origin  {lat0:.7f}°N  {lon0:.7f}°E  alt {alt0:.1f} m",
        fontsize=12
    )

    # ── 왼쪽: 3D ──────────────────────────────────────────
    ax3 = fig.add_subplot(121, projection='3d')

    lc3 = Line3DCollection(segs, cmap='coolwarm', linewidth=1.5, alpha=0.85)
    lc3.set_array(t_mid)
    ax3.add_collection3d(lc3)

    sc3 = ax3.scatter(x, y, z, c=t_norm, cmap='coolwarm', s=sz,
                      depthshade=True, zorder=3)

    ax3.scatter([x[0]],  [y[0]],  [z[0]],  color='blue', s=120,
                marker='o', zorder=5, label='Start')
    ax3.scatter([x[-1]], [y[-1]], [z[-1]], color='red',  s=120,
                marker='*', zorder=5, label='End')

    for xi, yi, zi in zip(x[::max(1,len(x)//40)],
                           y[::max(1,len(y)//40)],
                           z[::max(1,len(z)//40)]):
        ax3.plot([xi,xi],[yi,yi],[z_floor,zi],
                 color='gray', linewidth=0.4, alpha=0.25)

    ax3.set_xlim(x.min()-pad, x.max()+pad)
    ax3.set_ylim(y.min()-pad, y.max()+pad)
    ax3.set_zlim(z_floor,     z.max()+pad)
    ax3.set_xlabel('East (m)',  labelpad=8)
    ax3.set_ylabel('North (m)', labelpad=8)
    ax3.set_zlabel('Up (m)',    labelpad=8)
    ax3.set_title('3D View', fontsize=11)
    ax3.legend(fontsize=9)

    # ── 오른쪽: 2D XY projection ──────────────────────────
    from matplotlib.collections import LineCollection as LC2

    ax2 = fig.add_subplot(122, aspect='equal')

    segs2 = [[[x[k],y[k]],[x[k+1],y[k+1]]] for k in range(len(x)-1)]
    lc2   = LC2(segs2, cmap='coolwarm', linewidth=2.0, alpha=0.9)
    lc2.set_array(t_mid)
    ax2.add_collection(lc2)

    sc2 = ax2.scatter(x, y, c=t_norm, cmap='coolwarm', s=sz, zorder=3)

    ax2.scatter(x[0],  y[0],  color='blue', s=120, marker='o',
                zorder=5, label='Start')
    ax2.scatter(x[-1], y[-1], color='red',  s=120, marker='*',
                zorder=5, label='End')

    # altitude annotation at sparse points
    step = max(1, len(x)//20)
    for xi, yi, zi in zip(x[::step], y[::step], z[::step]):
        ax2.annotate(f'{zi:+.2f}m', (xi, yi),
                     fontsize=6, color='dimgray',
                     xytext=(3,3), textcoords='offset points')

    ax2.set_xlim(x.min()-pad, x.max()+pad)
    ax2.set_ylim(y.min()-pad, y.max()+pad)
    ax2.set_xlabel('East (m)')
    ax2.set_ylabel('North (m)')
    ax2.set_title('2D Top-down (XY) Projection', fontsize=11)
    ax2.legend(fontsize=9)
    ax2.grid(True, linewidth=0.5, alpha=0.5)

    # shared colorbar
    cbar = fig.colorbar(sc2, ax=[ax3, ax2], pad=0.04,
                        shrink=0.55, aspect=25, location='bottom')
    cbar.set_label('Time  (blue = start  →  red = end)', fontsize=10)

    plt.tight_layout()
    out_file = 'trajectory.png'
    plt.savefig(out_file, dpi=150, bbox_inches='tight')
    print(f"Saved → {out_file}")
    plt.show()