import datetime
import threading
import time
import logging
import yaml
import os

log = logging.getLogger(__name__)


class Scheduler:
    """
    Reads farm_config.yaml on each cycle to pick up remote config changes.
    Fires start_cb(waypoints) at scheduled times unless today is an off-day.
    Config keys (under 'schedule'):
      times:    ["11:00", "16:00"]
      off_days: ["2025-12-25", "2025-01-01"]
      enabled:  true
    """

    def __init__(self, config_path: str, start_cb):
        self._cfg_path = os.path.expanduser(config_path)
        self._start_cb = start_cb
        self._fired_today: set[str] = set()

    def run(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            now = datetime.datetime.now()
            date_str = now.strftime("%Y-%m-%d")
            time_str = now.strftime("%H:%M")

            # Reset fired set at midnight
            if time_str == "00:00":
                self._fired_today.clear()

            cfg = self._load_schedule()
            if cfg.get("enabled", True):
                today = now.strftime("%Y-%m-%d")
                if today not in cfg.get("off_days", []):
                    for t in cfg.get("times", []):
                        key = f"{date_str}_{t}"
                        if time_str == t and key not in self._fired_today:
                            self._fired_today.add(key)
                            log.info(f"Scheduled run at {t}")
                            self._start_cb(cfg.get("waypoints", []))

            time.sleep(30)

    def _load_schedule(self) -> dict:
        try:
            with open(self._cfg_path) as f:
                cfg = yaml.safe_load(f)
            return cfg.get("schedule", {})
        except Exception as e:
            log.error(f"Config read error: {e}")
            return {}
