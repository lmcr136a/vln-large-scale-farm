#!/usr/bin/env python3
"""
restart_f9p.py
  Cold-restarts the u-blox F9P GNSS receiver via UBX-CFG-RST.

  Use when the receiver is stuck at SV:0 (tracking no satellites) after an
  unclean power-off (e.g. battery ran out). A hard power loss can leave the
  battery-backed RAM (BBR: ephemeris/almanac/RTC/last-position) stale or
  corrupted, so the receiver fails to reacquire. A cold start wipes the BBR
  and forces a fresh satellite search.

  Usage:
    python3 restart_f9p.py            # cold start (clear all BBR)  [default]
    python3 restart_f9p.py --warm     # warm start (keep ephemeris)
    python3 restart_f9p.py --hot      # hot start  (keep all BBR, just reboot)
"""

import argparse
import struct
import sys
import time

import serial

F9P_PORT = '/dev/ttyACM0'
BAUD_F9P = 115200


def _ubx_checksum(msg: bytes) -> bytes:
    ck_a = ck_b = 0
    for b in msg:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])


def _ubx_packet(cls: int, mid: int, payload: bytes = b'') -> bytes:
    msg = bytes([cls, mid]) + struct.pack('<H', len(payload)) + payload
    return b'\xb5\x62' + msg + _ubx_checksum(msg)


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--warm', action='store_true', help='warm start (keep ephemeris)')
    g.add_argument('--hot',  action='store_true', help='hot start (keep all BBR)')
    ap.add_argument('--port', default=F9P_PORT)
    args = ap.parse_args()

    if args.hot:
        nav_bbr_mask, label = 0x0000, 'HOT  (keep all BBR)'
    elif args.warm:
        nav_bbr_mask, label = 0x0001, 'WARM (clear ephemeris only)'
    else:
        nav_bbr_mask, label = 0xFFFF, 'COLD (clear all BBR)'

    # UBX-CFG-RST: navBbrMask(U2), resetMode(U1), reserved(U1)
    # resetMode 0x01 = controlled software reset (GNSS + restart)
    payload = struct.pack('<HBB', nav_bbr_mask, 0x01, 0x00)
    pkt = _ubx_packet(0x06, 0x04, payload)

    try:
        ser = serial.Serial(args.port, BAUD_F9P, timeout=1)
    except serial.SerialException as e:
        print(f'[restart_f9p] cannot open {args.port}: {e}')
        print('  Is rtk_gps_node still running and holding the port? Stop it first.')
        sys.exit(1)

    print(f'[restart_f9p] sending {label} reset to {args.port} ...')
    ser.write(pkt)
    ser.flush()
    # CFG-RST is not acknowledged; the receiver reboots immediately.
    time.sleep(2.0)
    ser.close()
    print('[restart_f9p] reset sent. Receiver is rebooting.')
    print('  Give it a clear sky view; cold reacquisition takes ~30-90 s.')
    print('  Then start the GPS node again (it reconfigures the F9P on startup).')


if __name__ == '__main__':
    main()
