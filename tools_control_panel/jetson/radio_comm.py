import collections
import serial
import json
import threading
import time
import logging

log = logging.getLogger(__name__)
MAX_BYTES     = 1900
PING_INTERVAL = 10.0   # seconds between pings
PING_HISTORY  = 6      # keep last 6 results (= last 60 s)


class RadioComm:
    """
    Bidirectional serial radio channel: Jetson <-> Local PC.
    TX: telemetry + events.
    RX: commands + pong replies.

    Quality (0-100): pong success rate over the last PING_HISTORY pings.
      - Sends a ping every PING_INTERVAL seconds.
      - radio_bridge_linux.py must reply with {"type":"pong","t":<echo>}.
      - If no pong arrives before the next ping fires, that round is marked failed.
    """

    def __init__(self, port: str, baud: int, on_command=None):
        self._ser = serial.Serial(
            port=port, baudrate=baud,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, rtscts=True, timeout=1,
        )
        self._on_command = on_command
        self._tx_lock    = threading.Lock()
        self._running    = False

        self._ping_lock    = threading.Lock()
        self._ping_history: collections.deque = collections.deque()
        self._pending_ping: float | None = None  # timestamp of unanswered ping
        self._last_rtt_ms: float | None = None   # last measured round-trip time

    def start(self):
        self._running = True
        threading.Thread(target=self._rx_loop,   daemon=True).start()
        threading.Thread(target=self._ping_loop, daemon=True).start()
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

    def get_quality(self) -> int:
        """Return 0-100 based on pong success rate over last PING_HISTORY pings."""
        with self._ping_lock:
            if not self._ping_history:
                return 0
            success = sum(1 for _, ok in self._ping_history if ok)
            return int(success / len(self._ping_history) * 100)

    def get_rtt_ms(self) -> float | None:
        """Return last measured round-trip time in ms, or None if not yet measured."""
        return self._last_rtt_ms

    # ── Ping loop ─────────────────────────────────────────────
    def _ping_loop(self):
        time.sleep(3.0)  # let connection stabilize before first ping
        while self._running:
            with self._ping_lock:
                # Previous ping unanswered → mark as failed
                if self._pending_ping is not None:
                    self._record_ping(False)
                t = time.time()
                self._pending_ping = t
            self.send({"type": "ping", "t": t})
            time.sleep(PING_INTERVAL)

    def _record_ping(self, success: bool):
        """Must be called with _ping_lock held."""
        self._ping_history.append((time.time(), success))
        while len(self._ping_history) > PING_HISTORY:
            self._ping_history.popleft()

    # ── RX loop ───────────────────────────────────────────────
    def _rx_loop(self):
        while self._running:
            try:
                line = self._ser.readline()
                if not line:
                    continue
                msg = json.loads(line.decode('utf-8', errors='ignore').strip())

                if msg.get('type') == 'pong':
                    with self._ping_lock:
                        if self._pending_ping is not None:
                            self._last_rtt_ms = (time.time() - self._pending_ping) * 1000
                            self._record_ping(True)
                            self._pending_ping = None
                elif self._on_command:
                    self._on_command(msg)

            except json.JSONDecodeError:
                pass
            except Exception as e:
                if self._running:
                    log.error(f"Radio RX: {e}")
                    time.sleep(0.1)