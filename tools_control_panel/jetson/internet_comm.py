import base64
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

# RTT thresholds (seconds)
RTT_HIGH   = 0.08   # < 80ms  → high
RTT_MEDIUM = 0.25   # < 250ms → medium, else low
QUALITY_CHECK_INTERVAL = 15.0


class InternetComm:
    def __init__(self, lab_url: str, on_command=None):
        self._url        = lab_url
        self._on_command = on_command
        self._quality    = "medium"
        self._last_rgb: dict[str, float] = {}
        self._sio        = socketio.Client(reconnection=True, reconnection_delay=5, logger=False)
        self._connected  = False
        self._setup_events()

    @property
    def connected(self) -> bool:
        return self._connected

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
        def on_pong(data):
            pass  # handled inline in _measure_rtt

    def start(self):
        threading.Thread(target=self._connect_loop, daemon=True).start()

    def _connect_loop(self):
        while True:
            if not self._connected:
                try:
                    self._sio.connect(self._url, namespaces=["/"])
                except Exception as e:
                    log.warning(f"Internet connect failed: {e}, retry in 10s")
                    time.sleep(10)
                    continue
                # connect()가 예외 없이 리턴하거나 on_connect 콜백으로 _connected=True 된 경우
                threading.Thread(target=self._quality_loop, daemon=True).start()
                self._sio.wait()

    def _quality_loop(self):
        """Periodically measure RTT and auto-select quality level."""
        time.sleep(2.0)  # wait for connection to stabilize
        while self._connected:
            rtt = self._measure_rtt()
            if rtt is None:
                level = "low"
            elif rtt < RTT_HIGH:
                level = "high"
            elif rtt < RTT_MEDIUM:
                level = "medium"
            else:
                level = "low"

            if level != self._quality:
                self._quality = level
                log.info(f"Quality auto: {level} (RTT {rtt*1000:.0f}ms)" if rtt else f"Quality auto: {level} (no response)")

            time.sleep(QUALITY_CHECK_INTERVAL)

    def _measure_rtt(self) -> float | None:
        """Send a ping event and measure round-trip time. Returns seconds or None."""
        result = [None]
        done   = threading.Event()

        def handle_pong(data):
            result[0] = time.time()
            done.set()

        try:
            self._sio.on("pong_rtt", handle_pong, namespace="/")
            t0 = time.time()
            self._sio.emit("ping_rtt", {}, namespace="/")
            done.wait(timeout=3.0)
            if result[0] is not None:
                return result[0] - t0
            return None
        except Exception:
            return None