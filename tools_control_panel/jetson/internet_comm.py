import base64
import logging
import threading
import time

import numpy as np
import socketio  # pip install "python-socketio[client]"

log = logging.getLogger(__name__)

QUALITY = {
    "low":    {"rgb_hz": 0.1, "pc_ratio": 0.08},
    "medium": {"rgb_hz": 1.0, "pc_ratio": 0.20},
    "high":   {"rgb_hz": 5.0, "pc_ratio": 0.40},
}


class InternetComm:
    """
    Socket.IO client connecting to Lab PC remote_server (/internet namespace).
    Works over Tailscale — no Local PC bridge needed.
    Sends: telemetry, RGB frames (adaptive Hz), point cloud (adaptive sampling).
    Receives: quality level changes, commands.
    """

    def __init__(self, lab_url: str, on_command=None):
        # lab_url: http(s)://100.x.x.x:8000
        self._url = lab_url
        self._on_command = on_command
        self._quality = "low"
        self._last_rgb: dict[str, float] = {}
        self._sio = socketio.Client(reconnection=True, reconnection_delay=5, logger=False)
        self._setup_events()

    @property
    def connected(self) -> bool:
        return self._sio.connected

    # ── Public send API ───────────────────────────────────────────────────────

    def send_telemetry(self, payload: dict):
        self._emit("telemetry", payload)

    def send_event(self, event: str, data):
        self._emit("robot_event", {"event": event, "data": data})

    def send_rgb(self, frame_jpeg: bytes, camera: str):
        q = QUALITY[self._quality]
        interval = 1.0 / q["rgb_hz"] if q["rgb_hz"] > 0 else float("inf")
        if time.time() - self._last_rgb.get(camera, 0.0) < interval:
            return
        self._last_rgb[camera] = time.time()
        self._emit("rgb_frame", {
            "camera": camera,
            "data": base64.b64encode(frame_jpeg).decode(),
        })

    def send_pointcloud(self, points: np.ndarray):
        ratio = QUALITY[self._quality]["pc_ratio"]
        n = max(1, int(len(points) * ratio))
        idx = np.random.choice(len(points), n, replace=False)
        sampled = points[idx].astype(np.float32)
        self._emit("pointcloud", {
            "n": n,
            "data": base64.b64encode(sampled.tobytes()).decode(),
        })

    # ── Internal ──────────────────────────────────────────────────────────────

    def _emit(self, event: str, data):
        if not self._sio.connected:
            return
        try:
            self._sio.emit(event, data, namespace="/internet")
        except Exception as e:
            log.error(f"Internet emit {event}: {e}")

    def _setup_events(self):
        sio = self._sio

        @sio.on("connect", namespace="/internet")
        def on_connect():
            log.info(f"Internet connected to {self._url}")

        @sio.on("disconnect", namespace="/internet")
        def on_disconnect():
            log.warning("Internet disconnected")

        @sio.on("quality", namespace="/internet")
        def on_quality(data):
            level = data.get("level", "low")
            if level in QUALITY:
                self._quality = level
                log.info(f"Quality: {level}")

        @sio.on("command", namespace="/internet")
        def on_command(data):
            if self._on_command:
                self._on_command(data)

    def start(self):
        threading.Thread(target=self._connect_loop, daemon=True).start()

    def _connect_loop(self):
        while True:
            if not self._sio.connected:
                try:
                    self._sio.connect(self._url, namespaces=["/internet"])
                    self._sio.wait()
                except Exception as e:
                    log.warning(f"Internet connect failed: {e}, retry in 10s")
                    time.sleep(10)
