"""
RTK convergence diagnostic. Polls UBX-NAV-PVT and UBX-NAV-SIG to show
exactly why the F9P is stuck on Float instead of escalating to Fixed.

Run this WHILE gps_test.py is running (they don't share the f9p handle
since we open a fresh one).
"""
import serial, struct, time, sys

F9P_PORT = '/dev/ttyACM0'
BAUD     = 115200
POLL_EVERY = 2.0

GNSS = {0:'GPS', 1:'SBAS', 2:'GAL', 3:'BDS', 5:'QZS', 6:'GLO'}
CARR = {0:'None', 1:'Float', 2:'FIXED'}

def cs(msg):
    a = b = 0
    for x in msg:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes([a, b])

def pkt(cls, mid, payload=b''):
    m = bytes([cls, mid]) + struct.pack('<H', len(payload)) + payload
    return b'\xb5\x62' + m + cs(m)

def find_ubx(buf, cls, mid):
    """Find a complete UBX frame of given class/id in buf. Returns payload or None."""
    needle = bytes([0xb5, 0x62, cls, mid])
    i = buf.find(needle)
    if i < 0 or len(buf) < i + 8:
        return None
    length = struct.unpack_from('<H', buf, i + 4)[0]
    if len(buf) < i + 8 + length:
        return None
    return buf[i + 6 : i + 6 + length]

def read_until(f, cls, mid, timeout=1.5):
    deadline = time.time() + timeout
    buf = b''
    while time.time() < deadline:
        buf += f.read(2048)
        p = find_ubx(buf, cls, mid)
        if p is not None:
            return p
    return None

def parse_nav_pvt(p):
    if len(p) < 92:
        return None
    flags = p[21]
    return {
        'fixType':   p[20],
        'gnssFixOK': bool(flags & 0x01),
        'carrSoln':  (flags >> 6) & 0x03,
        'numSV':     p[23],
        'hAcc_mm':   struct.unpack_from('<I', p, 40)[0],
        'vAcc_mm':   struct.unpack_from('<I', p, 44)[0],
        'pDOP':      struct.unpack_from('<H', p, 76)[0] * 0.01,
    }

def parse_nav_sig(p):
    if len(p) < 8:
        return []
    num = p[5]
    out = []
    for i in range(num):
        off = 8 + i * 16
        if off + 16 > len(p):
            break
        gnssId  = p[off]
        svId    = p[off + 1]
        sigId   = p[off + 2]
        cno     = p[off + 4]
        quality = p[off + 5]
        corrSrc = p[off + 6]
        sigFlags = struct.unpack_from('<H', p, off + 8)[0]
        out.append({
            'gnss': GNSS.get(gnssId, f'?{gnssId}'),
            'sv': svId, 'sigId': sigId, 'cno': cno,
            'quality': quality, 'corrSrc': corrSrc,
            'prUsed':  bool(sigFlags & (1 << 3)),
            'crUsed':  bool(sigFlags & (1 << 4)),  # carrier-range used
            'doUsed':  bool(sigFlags & (1 << 5)),
        })
    return out

def sig_label(gnss, sigId):
    table = {
        ('GPS', 0): 'L1CA', ('GPS', 3): 'L2CL', ('GPS', 4): 'L2CM',
        ('GAL', 0): 'E1C',  ('GAL', 1): 'E1B',  ('GAL', 5): 'E5bI', ('GAL', 6): 'E5bQ',
        ('BDS', 0): 'B1I',  ('BDS', 1): 'B1I',  ('BDS', 2): 'B2I',  ('BDS', 3): 'B2I',
        ('GLO', 0): 'L1OF', ('GLO', 2): 'L2OF',
        ('SBAS', 0): 'L1CA',
        ('QZS', 0): 'L1CA', ('QZS', 4): 'L2CM', ('QZS', 5): 'L2CL',
    }
    return table.get((gnss, sigId), f'sig{sigId}')

def main():
    f = serial.Serial(F9P_PORT, BAUD, timeout=0.5)

    while True:
        # NAV-PVT
        f.reset_input_buffer()
        f.write(pkt(0x01, 0x07))
        p = read_until(f, 0x01, 0x07)
        pvt = parse_nav_pvt(p) if p else None

        # NAV-SIG
        f.reset_input_buffer()
        f.write(pkt(0x01, 0x43))
        p = read_until(f, 0x01, 0x43, timeout=2.0)
        sigs = parse_nav_sig(p) if p else []

        print('\033[2J\033[H', end='')  # clear screen
        if pvt:
            print(f"NAV-PVT:  fixType={pvt['fixType']}  carrSoln={CARR[pvt['carrSoln']]}  "
                  f"numSV={pvt['numSV']}  pDOP={pvt['pDOP']:.2f}")
            print(f"          hAcc={pvt['hAcc_mm']/1000:.3f}m   vAcc={pvt['vAcc_mm']/1000:.3f}m")
            if pvt['carrSoln'] == 2:
                print("          ===> RTK FIXED <===")
            elif pvt['carrSoln'] == 1:
                print("          (Float - waiting for ambiguity resolution)")
        else:
            print("NAV-PVT: no response")

        if sigs:
            # Summary by constellation x band
            print(f"\nNAV-SIG: {len(sigs)} signals tracked")
            print(f"  {'GNSS':<6}{'SV':<4}{'BAND':<6}{'C/N0':<6}{'qual':<6}{'pr':<4}{'cr':<4}{'do':<4}")
            for s in sorted(sigs, key=lambda x: (x['gnss'], x['sv'], x['sigId'])):
                band = sig_label(s['gnss'], s['sigId'])
                cn = f"{s['cno']:>3}" + ('*' if s['cno'] >= 40 else ' ')
                pr = 'Y' if s['prUsed'] else '.'
                cr = 'Y' if s['crUsed'] else '.'
                do = 'Y' if s['doUsed'] else '.'
                print(f"  {s['gnss']:<6}{s['sv']:<4}{band:<6}{cn:<6}{s['quality']:<6}{pr:<4}{cr:<4}{do:<4}")

            # Critical RTK metrics
            l2_signals = [s for s in sigs if sig_label(s['gnss'], s['sigId'])
                          in ('L2CL', 'L2CM', 'L2OF', 'E5bI', 'E5bQ', 'B2I')]
            cr_used = [s for s in sigs if s['crUsed']]
            strong  = [s for s in sigs if s['cno'] >= 40]
            l2_strong_cr = [s for s in l2_signals if s['cno'] >= 35 and s['crUsed']]

            print(f"\nRTK readiness:")
            print(f"  L2/E5b/B2 tracked:        {len(l2_signals):>3}  "
                  f"(need >= 5 for reliable Fixed)")
            print(f"  L2-band with C/N0>=35:    {len(l2_strong_cr):>3}  "
                  f"(if low: poor L2 tracking - antenna/multipath)")
            print(f"  Carrier-range used:       {len(cr_used):>3}  "
                  f"(signals contributing to RTK)")
            print(f"  C/N0 >= 40 dBHz:          {len(strong):>3}  "
                  f"(strong signals; low = sky obstructed or bad antenna)")
        else:
            print("\nNAV-SIG: no response")

        time.sleep(POLL_EVERY)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass