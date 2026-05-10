import serial, json
from datetime import datetime

s = serial.Serial('/dev/ttyTHS1', 57600, timeout=1)
print(f"{'TIME':8} {'MODE':8} {'ESTOP':6} {'POS_X':7} {'POS_Y':7} {'YAW_DEG':8} {'SENSORS':20}")
print('-' * 75)

while True:
    line = s.readline()
    if not line:
        continue
    try:
        msg = json.loads(line.decode().strip())
        d   = msg['data']
        p   = d.get('pose', [0]*7)
        import math
        qz, qw = p[5], p[6]
        yaw = math.degrees(math.atan2(2*(qw*qz), 1 - 2*qz**2))
        sens = d.get('sensors', {})
        sens_str = ' '.join(f"{'✓' if v else '✗'}{k[:3]}" for k,v in sens.items())
        print(f"{datetime.now().strftime('%H:%M:%S')}  "
              f"{d.get('mode','?'):8} "
              f"{'YES' if d.get('estop') else 'no':6} "
              f"{p[0]:7.3f} {p[1]:7.3f} {yaw:8.1f}° "
              f"{sens_str}")
    except Exception:
        print(f"  [CORRUPT] {line[:50]}")