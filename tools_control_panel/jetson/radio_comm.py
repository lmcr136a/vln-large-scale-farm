import collections
import serial
import json
import threading
import time
import logging

log = logging.getLogger(__name__)
MAX_BYTES = 1900

# Quality measurement window (seconds)
_QUALITY_WINDOW = 30.0
_RX_TIMEOUT     = 10.0   # no packet for 10s → rx_score = 0


class RadioComm:
    """
    Bidirectional serial radio channel: Jetson <-> Local PC.
    TX: telemetry + events (pose, status, sensor health).
    RX: commands (estop, velocity, continue, new_path, config_update).
    Quality score (0-100) derived from TX success rate + RX heartbeat activity.
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

        # TX sliding window: deque of (timestamp, success: bool)
        self._tx_window: collections.deque = collections.deque()
        self._tx_window_lock = threading.Lock()

        # Last time an RX packet was received
        self._last_rx_time: float = 0.0

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
            self._record_tx(False)
            return False
        with self._tx_lock:
            try:
                self._ser.write(data)
                self._record_tx(True)
                return True
            except Exception as e:
                if self._running:
                    log.error(f"Radio TX: {e}")
                self._record_tx(False)
                return False

    def get_quality(self) -> int:
        """Return 0-100 radio link quality score (Jetson <-> Local PC)."""
        now = time.time()
        cutoff = now - _QUALITY_WINDOW

        with self._tx_window_lock:
            # Evict old entries
            while self._tx_window and self._tx_window[0][0] < cutoff:
                self._tx_window.popleft()
            total = len(self._tx_window)
            ok    = sum(1 for _, s in self._tx_window if s)

        # TX success rate (0.0-1.0)
        tx_score = (ok / total) if total > 0 else 0.0

        # RX activity: decay to 0 if no packet received within _RX_TIMEOUT
        elapsed_rx = now - self._last_rx_time if self._last_rx_time > 0 else _QUALITY_WINDOW
        rx_score = max(0.0, 1.0 - elapsed_rx / _RX_TIMEOUT)

        # If TX is completely fresh (no history yet), trust RX side only
        if total == 0:
            return int(rx_score * 100)

        return int((tx_score * 0.65 + rx_score * 0.35) * 100)

    def _record_tx(self, success: bool):
        with self._tx_window_lock:
            self._tx_window.append((time.time(), success))

    def _rx_loop(self):
        while self._running:
            try:
                line = self._ser.readline()
                if not line:
                    continue
                self._last_rx_time = time.time()
                cmd = json.loads(line.decode('utf-8', errors='ignore').strip())
                if self._on_command:
                    self._on_command(cmd)
            except json.JSONDecodeError:
                pass
            except Exception as e:
                if self._running:
                    log.error(f"Radio RX: {e}")
                    time.sleep(0.1)
