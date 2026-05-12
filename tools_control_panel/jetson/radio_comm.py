import serial
import json
import threading
import time
import logging

log = logging.getLogger(__name__)
MAX_BYTES = 1900


class RadioComm:
    """
    Bidirectional serial radio channel: Jetson <-> Local PC.
    TX: telemetry + events (pose, status, sensor health).
    RX: commands (estop, velocity, continue, new_path, config_update).
    """

    def __init__(self, port: str, baud: int, on_command=None):
        self._ser = serial.Serial(
            port=port, baudrate=baud,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, rtscts=True, timeout=1,
        )
        self._on_command = on_command
        self._tx_lock = threading.Lock()
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._rx_loop, daemon=True).start()
        log.info(f"Radio opened: {self._ser.port}")

    def stop(self):
        self._running = False
        self._ser.close()

    def send(self, payload: dict) -> bool:
        if not self._running:
            return False
        data = (json.dumps(payload, separators=(',', ':')) + '\n').encode()
        if len(data) > MAX_BYTES:
            log.warning(f"Packet too large ({len(data)}B), dropping")
            return False
        with self._tx_lock:
            try:
                self._ser.write(data)
                return True
            except Exception as e:
                if self._running:
                    log.error(f"Radio TX: {e}")
                return False

    def _rx_loop(self):
        while self._running:
            try:
                line = self._ser.readline()
                if not line:
                    continue
                cmd = json.loads(line.decode('utf-8', errors='ignore').strip())
                if self._on_command:
                    self._on_command(cmd)
            except json.JSONDecodeError:
                pass
            except Exception as e:
                if self._running:
                    log.error(f"Radio RX: {e}")
                    time.sleep(0.1)