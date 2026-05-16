import serial, struct, time

def cs(msg):
    a = b = 0
    for x in msg:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes([a, b])

def pkt(cls, mid, payload=b''):
    m = bytes([cls, mid]) + struct.pack('<H', len(payload)) + payload
    return b'\xb5\x62' + m + cs(m)

f = serial.Serial('/dev/ttyACM0', 115200, timeout=2.0)
f.reset_input_buffer()

# Layers = 7 (RAM | BBR | Flash) so it sticks across reboots
payload = struct.pack('<BBH', 0, 7, 0)
keys = [
    (0x10780001, 1),  # CFG-USBOUTPROT-UBX
    (0x10780002, 1),  # CFG-USBOUTPROT-NMEA
    (0x10770001, 1),  # CFG-USBINPROT-UBX
    (0x10770004, 1),  # CFG-USBINPROT-RTCM3X
]
for k, v in keys:
    payload += struct.pack('<IB', k, v)
f.write(pkt(0x06, 0x8A, payload))
print("VALSET sent (layers=RAM+BBR+Flash). Waiting for the F9P to start outputting UBX...")
time.sleep(1.5)

# Probe MON-VER
f.reset_input_buffer()
f.write(pkt(0x0A, 0x04))
time.sleep(0.7)
data = f.read(8192)
has_ubx = b'\xb5\x62\x0a\x04' in data
print(f"\nMON-VER probe: got {len(data)} bytes, UBX present: {has_ubx}")
if has_ubx:
    i = data.find(b'\xb5\x62\x0a\x04')
    print(f"Firmware string: {data[i+6:i+6+40]}")
    print("\nF9P USB-UBX is now ALIVE. The fix is saved to flash.")
else:
    print("\nF9P still not responding with UBX. Possible remaining issues:")
    print("  - USBINPROT-UBX was also disabled (F9P can't even receive UBX)")
    print("  - Need to use UART pins instead of USB to recover")
    print("  - Try u-center via UART, or hard-reset the F9P")
f.close()