"""
Station uploader.
Triggered when autonomous run completes and robot pose is within
upload.station_radius of upload.station_pose AND WiFi is available.
Uploads via SCP (SSH key auth, no password).
"""
import logging
import math
import os
import subprocess
import threading
import time

import yaml

log = logging.getLogger(__name__)


def _wifi_ssid() -> str:
    try:
        return os.popen("iwgetid -r").read().strip()
    except Exception:
        return ""


def _at_station(pose: list, cfg: dict) -> bool:
    sx, sy = cfg["station_pose"]
    dx = pose[0] - sx
    dy = pose[1] - sy
    return math.sqrt(dx * dx + dy * dy) <= cfg["station_radius"]


class StationUploader:
    """
    Call notify_run_completed(pose) when an autonomous run finishes.
    If pose is at station and WiFi is up, SCP upload starts in background.
    Repeated triggers within cooldown_sec are ignored.
    """

    def __init__(self, config_path: str):
        self._config_path = os.path.expanduser(config_path)
        self._lock = threading.Lock()
        self._last_upload = 0.0
        self._uploading = False

    # ── Public ────────────────────────────────────────────────────────────────

    def notify_run_completed(self, pose: list):
        cfg = self._load_upload_cfg()
        if not cfg.get("enabled", True):
            return
        if not _at_station(pose, cfg):
            log.info("Run completed but not at station — skipping upload")
            return
        if not _wifi_ssid():
            log.info("At station but no WiFi — skipping upload")
            return
        with self._lock:
            since = time.time() - self._last_upload
            if self._uploading or since < cfg.get("cooldown_sec", 300):
                log.info(f"Upload skipped (uploading={self._uploading}, since={since:.0f}s)")
                return
            self._uploading = True
        threading.Thread(target=self._upload, args=(cfg,), daemon=True).start()

    # ── Internal ──────────────────────────────────────────────────────────────

    def _upload(self, cfg: dict):
        user    = cfg["ssh_user"]
        host    = cfg["host"]
        key     = os.path.expanduser(cfg["ssh_key"])
        remote  = cfg["remote_path"]
        sources = [os.path.expanduser(s) for s in cfg.get("sources", [])]

        log.info(f"Starting upload → {user}@{host}:{remote}")
        ok = True
        for src in sources:
            if not os.path.exists(src):
                log.warning(f"Source not found: {src}")
                continue
            cmd = [
                "scp", "-r",
                "-i", key,
                "-o", "StrictHostKeyChecking=no",
                "-o", "BatchMode=yes",        # fail immediately if key not accepted
                src,
                f"{user}@{host}:{remote}",
            ]
            log.info(f"scp {src} → {user}@{host}:{remote}")
            try:
                result = subprocess.run(cmd, timeout=600, capture_output=True, text=True)
                if result.returncode != 0:
                    log.error(f"scp failed: {result.stderr.strip()}")
                    ok = False
                else:
                    log.info(f"Upload OK: {src}")
            except subprocess.TimeoutExpired:
                log.error(f"scp timed out: {src}")
                ok = False
            except Exception as e:
                log.error(f"scp error: {e}")
                ok = False

        with self._lock:
            self._uploading = False
            if ok:
                self._last_upload = time.time()
                log.info("All uploads completed")
            else:
                log.warning("Upload finished with errors")

    def _load_upload_cfg(self) -> dict:
        try:
            with open(self._config_path) as f:
                return yaml.safe_load(f).get("upload", {})
        except Exception as e:
            log.error(f"Config read error: {e}")
            return {}