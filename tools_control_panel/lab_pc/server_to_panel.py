import json
import os
import shutil
import time

import psutil

_JETSON_MAP_TIMEOUT = 5.0   # seconds without a Jetson frame before falling back to saved map


class ServerToPanel:
    def __init__(self, socketio, config):
        self.socketio = socketio
        self.config   = config
        self._map_version     = 0
        self._last_mtime      = 0.0
        self._last_jetson_map = 0.0

        cfg_dir = os.path.dirname(os.path.abspath(__file__))
        proj_root = os.path.dirname(cfg_dir)
        raw = config['paths'].get('saved_map_dir', 'lab_pc/maps')
        self._saved_map_dir = os.path.normpath(os.path.join(proj_root, raw))
        os.makedirs(self._saved_map_dir, exist_ok=True)

    def notify_jetson_map(self):
        """Called by RemoteServer each time a live map_frame arrives from Jetson."""
        self._last_jetson_map = time.time()

    @property
    def _jetson_active(self) -> bool:
        return time.time() - self._last_jetson_map < _JETSON_MAP_TIMEOUT

    def send_saved_map(self, force=False):
        """Emit map_updated pointing to saved_map.png — used when Jetson is offline."""
        if self._jetson_active and not force:
            return
        png_path   = os.path.join(self._saved_map_dir, 'saved_map.png')
        state_path = os.path.join(self._saved_map_dir, 'saved_map_state.json')
        if not os.path.exists(png_path) or not os.path.exists(state_path):
            return
        mtime = os.path.getmtime(png_path)
        if not force and mtime <= self._last_mtime:
            return
        self._last_mtime = mtime
        try:
            with open(state_path) as f:
                meta = json.load(f)
        except Exception:
            return
        self._map_version += 1
        try:
            self.socketio.emit('map_updated', {
                'version':    self._map_version,
                'resolution': float(meta.get('resolution', 0.1)),
                'origin_x':   float(meta.get('origin_x',  0.0)),
                'origin_y':   float(meta.get('origin_y',  0.0)),
                'width':      int(meta.get('img_width',   0)),
                'height':     int(meta.get('img_height',  0)),
                'rot_angle':  float(meta.get('rot_angle', 0.0)),
                # no image_data → browser fetches /saved_map.png via HTTP
            })
        except Exception as e:
            print(f'[map] emit failed: {e}')

    def send_sysmon(self):
        data_dir = os.path.expanduser(self.config['paths']['data_dir'])
        os.makedirs(data_dir, exist_ok=True)
        total, used, _ = shutil.disk_usage(data_dir)
        wifi = ''
        try:
            wifi = os.popen('iwgetid -r').read().strip()
        except Exception:
            pass
        try:
            self.socketio.emit('sysmon', {
                'cpu':      psutil.cpu_percent(interval=None),
                'mem':      psutil.virtual_memory().percent,
                'used_gb':  round(used  / 1024**3, 1),
                'total_gb': round(total / 1024**3, 1),
                'used_pct': round(used  / total * 100, 1),
                'wifi':     wifi or 'Not connected',
            })
        except Exception:
            pass

    def monitor_loop(self):
        sysmon_interval = self.config['monitor']['update_interval']
        map_interval    = self.config.get('map', {}).get('update_interval', 1.0)
        last_sysmon = last_map = 0.0
        self.send_saved_map(force=True)   # send whatever we have on startup
        while True:
            now = time.time()
            if now - last_map >= map_interval:
                self.send_saved_map()
                last_map = now
            if now - last_sysmon >= sysmon_interval:
                self.send_sysmon()
                last_sysmon = now
            time.sleep(0.05)