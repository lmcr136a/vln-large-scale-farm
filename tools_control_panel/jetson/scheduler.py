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
            time_str = now.strftime("%H:%M")
            date_str = now.strftime("%Y-%m-%d")

            if time_str == "00:00":
                self._fired_today.clear()

            try:
                with open(self._cfg_path, encoding='utf-8') as f:
                    cfg = yaml.safe_load(f)
                sched     = cfg.get("schedule", {})
                waypoints = cfg.get("autonomous", {}).get("waypoints", [])
            except Exception as e:
                log.error(f"Config read error: {e}")
                time.sleep(30)
                continue

            if sched.get("enabled", True) and date_str not in sched.get("off_days", []):
                for t in sched.get("times", []):
                    key = f"{date_str}_{t}"
                    if time_str == t and key not in self._fired_today:
                        self._fired_today.add(key)
                        if len(waypoints) >= 2:
                            log.info(f"Scheduled run at {t} ({len(waypoints)} waypoints)")
                            self._start_cb(waypoints)
                        else:
                            log.warning(f"Scheduled run at {t} skipped — no waypoints set")

            time.sleep(30)