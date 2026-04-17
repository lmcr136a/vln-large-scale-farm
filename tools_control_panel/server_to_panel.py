"""
Server to Panel Communication.

Responsibilities:
  - sysmon:     CPU / memory / disk / WiFi  → every ~1s
  - map_updated: watch map_latest.png for changes (mtime), read yaml metadata,
                 emit metadata to client (client fetches PNG via HTTP GET)
  - robot_pose:  NOT handled here — autonomous_mode.py emits directly
"""
import os
import time
import shutil
import json
import psutil
from path_utils import resolve_configured_path


class ServerToPanel:
    def __init__(self, socketio, config):
        self.socketio       = socketio
        self.config         = config
        self._last_mtime    = 0.0
        self._map_version   = 0
        self.map_dir = resolve_configured_path(
            self.config['paths'].get('map_dir'),
            os.path.join('tools_control_panel', 'output_glim'),
        )
        self.data_dir = resolve_configured_path(
            self.config['paths'].get('data_dir'),
            'data',
        )

    # ── WiFi ──────────────────────────────────────────────────────────────────

    def get_wifi_name(self):
        try:
            result = os.popen("iwgetid -r").read().strip()
            return result if result else "Not connected"
        except Exception:
            return "Unavailable"

    # ── Map ───────────────────────────────────────────────────────────────────

    def _map_state_path(self):
        return os.path.join(self.map_dir, 'map_state.json')

    def send_map_update(self, force=False):
        """
        Check if map_latest.png has changed (via mtime).
        If yes, read map_state.json for metadata and emit map_updated.
        The client fetches the actual PNG via HTTP GET /map_latest.png?v=N.
        """
        png_path = os.path.join(self.map_dir, 'map_latest.png')
        state_path = self._map_state_path()

        if not os.path.exists(png_path) or not os.path.exists(state_path):
            return

        mtime = os.path.getmtime(png_path)
        if not force and mtime <= self._last_mtime:
            return
        self._last_mtime = mtime

        try:
            with open(state_path, 'r') as f:
                meta = json.load(f)
        except Exception:
            return

        self._map_version += 1

        try:
            self.socketio.emit('map_updated', {
                'version':    self._map_version,
                'resolution': float(meta.get('resolution', 0.1)),
                'origin_x':   float(meta.get('origin_x', 0.0)),
                'origin_y':   float(meta.get('origin_y', 0.0)),
                'width':      int(meta.get('img_width',  0)),
                'height':     int(meta.get('img_height', 0)),
                'rot_angle':  float(meta.get('rot_angle', 0.0)),
            })
            print(f'[map] notified {meta.get("img_width")}×{meta.get("img_height")}')
        except Exception as e:
            print(f'[map] emit failed: {e}')

    # ── System Monitor ────────────────────────────────────────────────────────

    def send_system_monitor(self, linear_speed):
        data_dir = self.data_dir
        os.makedirs(data_dir, exist_ok=True)

        total, used, _ = shutil.disk_usage(data_dir)
        cpu  = psutil.cpu_percent(interval=None)
        mem  = psutil.virtual_memory().percent
        wifi = self.get_wifi_name()

        try:
            self.socketio.emit('sysmon', {
                'cpu':        cpu,
                'mem':        mem,
                'used_gb':    round(used  / (1024**3), 1),
                'total_gb':   round(total / (1024**3), 1),
                'used_pct':   round((used / total) * 100, 1),
                'linear_mps': round(linear_speed, 2),
                'linear_mph': round(linear_speed * 2.23694, 2),
                'wifi':       wifi,
            })
        except Exception:
            pass

    # ── Main Loop ─────────────────────────────────────────────────────────────

    def monitor_loop(self, linear_speed_getter):
        sysmon_interval = self.config['monitor']['update_interval']
        map_interval    = self.config.get('map', {}).get('update_interval', 1.0)

        last_sysmon = 0.0
        last_map    = 0.0

        self.send_map_update(force=True)

        while True:
            now = time.time()

            if now - last_map >= map_interval:
                self.send_map_update()
                last_map = now

            if now - last_sysmon >= sysmon_interval:
                self.send_system_monitor(linear_speed_getter())
                last_sysmon = now

            time.sleep(0.05)
