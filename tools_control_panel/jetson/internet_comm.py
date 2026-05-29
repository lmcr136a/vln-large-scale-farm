import base64
import collections
import logging
import threading
import time

import numpy as np
import socketio

log = logging.getLogger(__name__)

QUALITY = {
    "low":    {"rgb_hz": 0.1, "pc_ratio": 0.08},
    "medium": {"rgb_hz": 1.0, "pc_ratio": 0.20},
    "high":   {"rgb_hz": 5.0, "pc_ratio": 0.40},
}

# RTT thresholds (seconds) for bandwidth-level selection
RTT_HIGH   = 0.08   # < 80ms  → high
RTT_MEDIUM = 0.25   # < 250ms → medium, else low

# Link probing: frequent pings + rolling window (mirrors RadioComm).
PING_INTERVAL = 3.0   # seconds between RTT pings
PING_HISTORY  = 10    # keep last 10 results (= last 30 s)


class InternetComm:
    def __init__(self, lab_url: str, on_command=None):
        self._url        = lab_url
        self._on_command = on_command
        self._quality    = "medium"
        self._last_rtt_ms: float | None = None
        self._last_rgb: dict[str, float] = {}
        self._sio        = socketio.Client(reconnection=True, reconnection_delay=5, logger=False)
        self._connected  = False
        self._ping_lock     = threading.Lock()
        self._ping_history  = collections.deque()
        self._pending_ping  = None  # timestamp of unanswered ping
        self._setup_events()

    @property
    def connected(self) -> bool:
        return self._connected

    def get_quality(self) -> int:
        """0-100 link score = pong success rate over the last PING_HISTORY pings."""
        if not self._connected:
            return 0
        with self._ping_lock:
            if not self._ping_history:
                return 0
            ok = sum(1 for _, success in self._ping_history if success)
            return int(ok / len(self._ping_history) * 100)

    def get_rtt_ms(self) -> float | None:
        return self._last_rtt_ms

    # ── Send API ──────────────────────────────────────────────────────────────

    def send_telemetry(self, payload: dict):
        self._emit("telemetry", payload)

    def send_pose(self, x: float, y: float, yaw: float):
        self._emit("pose", {"x": x, "y": y, "yaw": yaw})

    def send_event(self, event: str, data):
        self._emit("robot_event", {"event": event, "data": data})

    def send_rgb(self, frame_jpeg: bytes, camera: str):
        self._send_rgb_internal(base64.b64encode(frame_jpeg).decode(), camera)

    def send_rgb_b64(self, b64: str, camera: str):
        self._send_rgb_internal(b64, camera)

    def _send_rgb_internal(self, b64: str, camera: str):
        q = QUALITY[self._quality]
        interval = 1.0 / q["rgb_hz"] if q["rgb_hz"] > 0 else float("inf")
        if time.time() - self._last_rgb.get(camera, 0.0) < interval:
            return
        self._last_rgb[camera] = time.time()
        self._emit("rgb_frame", {"camera": camera, "data": b64})

    def send_pointcloud(self, points: np.ndarray):
        ratio = QUALITY[self._quality]["pc_ratio"]
        n     = max(1, int(len(points) * ratio))
        idx   = np.random.choice(len(points), n, replace=False)
        self._emit("pointcloud", {
            "n":    n,
            "data": base64.b64encode(points[idx].astype(np.float32).tobytes()).decode(),
        })


    def send_map(self, png_path: str, meta: dict):
        try:
            with open(png_path, "rb") as f:
                data = base64.b64encode(f.read()).decode()
            self._emit("map_frame", {"data": data, "meta": meta})
        except Exception as e:
            log.error(f"send_map: {e}")

    # ── Internal ──────────────────────────────────────────────────────────────

    def _emit(self, event: str, data):
        if not self._connected:
            return
        try:
            self._sio.emit(event, data, namespace="/")
        except Exception as e:
            log.error(f"emit {event}: {e}")

    def _setup_events(self):
        sio = self._sio

        @sio.on("connect", namespace="/")
        def on_connect():
            self._connected = True
            with self._ping_lock:
                self._ping_history.clear()
                self._pending_ping = None
            log.info(f"Internet connected: {self._url}")
            self._sio.emit("jetson_hello", {}, namespace="/")

        @sio.on("disconnect", namespace="/")
        def on_disconnect():
            self._connected = False
            log.warning("Internet disconnected")

        @sio.on("quality", namespace="/")
        def on_quality(data):
            level = data.get("level", "low")
            if level in QUALITY:
                self._quality = level
                log.info(f"Quality override: {level}")

        @sio.on("command", namespace="/")
        def on_command(data):
            if self._on_command:
                self._on_command(data)

        @sio.on("pong_rtt", namespace="/")
        def on_pong(data=None):
            with self._ping_lock:
                if self._pending_ping is None:
                    return
                rtt = time.time() - self._pending_ping
                self._last_rtt_ms = round(rtt * 1000, 1)
                self._record_ping(True)
                self._pending_ping = None
            self._update_level_from_rtt(rtt)

    def start(self):
        threading.Thread(target=self._connect_loop, daemon=True).start()
        threading.Thread(target=self._ping_loop,    daemon=True).start()

    def _connect_loop(self):
        while True:
            if not self._connected:
                try:
                    self._sio.connect(self._url, namespaces=["/"])
                except Exception as e:
                    if not self._connected:
                        log.warning(f"Internet connect failed: {e}, retry in 10s")
                        time.sleep(10)
                        continue
                    # on_connect fired before exception — connection is actually up
                    log.info("Connected (namespace warning ignored)")
            self._sio.wait()
            self._connected = False
            log.warning("Internet disconnected, retry in 5s")
            time.sleep(5)

    def _record_ping(self, success: bool):
        """Append a ping result. Must be called with _ping_lock held."""
        self._ping_history.append((time.time(), success))
        while len(self._ping_history) > PING_HISTORY:
            self._ping_history.popleft()

    def _update_level_from_rtt(self, rtt_s: float):
        """Select bandwidth level (rgb/pointcloud rate) from latest RTT."""
        if rtt_s < RTT_HIGH:
            self._quality = "high"
        elif rtt_s < RTT_MEDIUM:
            self._quality = "medium"
        else:
            self._quality = "low"

    def _ping_loop(self):
        """Ping every PING_INTERVAL; an unanswered ping counts as a failure."""
        time.sleep(2.0)
        while True:
            if not self._connected:
                with self._ping_lock:
                    self._pending_ping = None
                    self._ping_history.clear()
                time.sleep(PING_INTERVAL)
                continue
            with self._ping_lock:
                if self._pending_ping is not None:
                    self._record_ping(False)   # previous ping never answered
                    self._quality = "low"
                self._pending_ping = time.time()
            try:
                self._sio.emit("ping_rtt", {}, namespace="/")
            except Exception:
                with self._ping_lock:
                    self._record_ping(False)
                    self._pending_ping = None
            time.sleep(PING_INTERVAL)