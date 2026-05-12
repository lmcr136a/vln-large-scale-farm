import json
import os
import shutil
import time

import psutil


class ServerToPanel:
    def __init__(self, socketio, config):
        self.socketio     = socketio
        self.config       = config
        self._last_mtime  = 0.0
        self._map_version = 0
        self._saved_map_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'maps')
        os.makedirs(self._saved_map_dir, exist_ok=True)

    def _map_dir(self):
        return os.path.expanduser(self.config['paths']['map_dir'])

    def _map_image_path(self):
        return os.path.join(self._map_dir(), self.config['paths'].get('map_image', 'map_latest.png'))

    def _map_state_path(self):
        return os.path.join(self._map_dir(), self.config['paths'].get('map_state', 'map_state.json'))

    def send_map_update(self, force=False):
        png_path   = self._map_image_path()
        state_path = self._map_state_path()
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
            })
        except Exception as e:
            print(f'[map] emit failed: {e}')

    def send_sysmon(self):
        data_dir = os.path.expanduser(self.config['paths']['data_dir'])
        os.makedirs(data_dir, exist_ok=True)
        total, used, _ = shutil.disk_usage(data_dir)
        try:
            self.socketio.emit('sysmon', {
                'cpu':      psutil.cpu_percent(interval=None),
                'mem':      psutil.virtual_memory().percent,
                'used_gb':  round(used  / 1024**3, 1),
                'total_gb': round(total / 1024**3, 1),
                'used_pct': round(used  / total * 100, 1),
            })
        except Exception:
            pass

    def monitor_loop(self):
        sysmon_interval = self.config['monitor']['update_interval']
        map_interval    = self.config.get('map', {}).get('update_interval', 1.0)
        last_sysmon = last_map = 0.0
        self.send_map_update(force=True)
        while True:
            now = time.time()
            if now - last_map >= map_interval:
                self.send_map_update()
                last_map = now
            if now - last_sysmon >= sysmon_interval:
                self.send_sysmon()
                last_sysmon = now
            time.sleep(0.05)