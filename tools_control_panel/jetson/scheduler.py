import datetime
import json
import logging
import os
import re
import threading
import time
import urllib.request
import yaml

log = logging.getLogger(__name__)
log.setLevel(logging.DEBUG)

DAYS = ['mon', 'tue', 'wed', 'thu', 'fri', 'sat', 'sun']
# datetime.weekday(): 0=Mon … 5=Sat, 6=Sun
_WEEKDAY_MAP = {d: i for i, d in enumerate(DAYS)}


def _parse_time(s: str):
    """Parse '5am', '11pm' → (hour, minute). Returns None on error."""
    m = re.fullmatch(r'(\d{1,2})(am|pm)', s.strip().lower())
    if not m:
        return None
    h, period = int(m.group(1)), m.group(2)
    if period == 'pm' and h != 12:
        h += 12
    elif period == 'am' and h == 12:
        h = 0
    return h, 0


class Scheduler:
    """
    Reads schedule.json (same directory as farm_config.yaml).
    Fires start_cb(waypoints) at scheduled times, with retry on failure.
    """

    def __init__(self, config_path: str, start_cb, lab_url: str = None):
        self._cfg_path      = os.path.expanduser(config_path)
        self._schedule_file = os.path.join(os.path.dirname(self._cfg_path), 'schedule.json')
        self._start_cb      = start_cb
        self._lab_url       = lab_url.rstrip('/') if lab_url else None
        self._fired_today: set = set()

    def run(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _load_schedule(self) -> dict:
        # Try Lab PC API first
        if self._lab_url:
            try:
                with urllib.request.urlopen(f'{self._lab_url}/schedule', timeout=3) as r:
                    return json.loads(r.read())
            except Exception as e:
                log.warning(f'Schedule fetch from Lab PC failed: {e}, falling back to local')
        # Fallback: local file
        try:
            with open(self._schedule_file) as f:
                return json.load(f)
        except FileNotFoundError:
            return {'enabled': False}
        except Exception as e:
            log.error(f'Schedule read error: {e}')
            return {'enabled': False}

    def _load_waypoints(self) -> list:
        try:
            with open(self._cfg_path, encoding='utf-8') as f:
                cfg = yaml.safe_load(f)
            mission_file = cfg.get('paths', {}).get('mission_file', 'mission.json')
            if not os.path.isabs(mission_file):
                mission_file = os.path.join(os.path.dirname(self._cfg_path), mission_file)
            with open(mission_file, encoding='utf-8') as f:
                mission = json.load(f)
            return mission.get('waypoints', [])
        except FileNotFoundError:
            log.warning('mission.json not found')
            return []
        except Exception as e:
            log.error(f'Waypoints load error: {e}')
            return []

    def _loop(self):
        while True:
            try:
                now      = datetime.datetime.now()
                hhmm     = (now.hour, now.minute)
                date_str = now.strftime('%Y-%m-%d')
                weekday  = now.weekday()

                if now.strftime('%H:%M') == '00:00':
                    self._fired_today.clear()

                sched = self._load_schedule()
                enabled = sched.get('enabled', True)
                day_key = DAYS[weekday]
                times   = sched.get(day_key, [])
                log.debug(f'Scheduler check {now.strftime("%H:%M")} enabled={enabled} {day_key}={times}')

                if enabled:
                    for time_str in times:
                        parsed = _parse_time(time_str)
                        if not parsed:
                            log.warning(f'Cannot parse time: {time_str!r}')
                            continue
                        if hhmm == parsed:
                            fire_key = f'{date_str}_{time_str}'
                            if fire_key not in self._fired_today:
                                self._fired_today.add(fire_key)
                                waypoints = self._load_waypoints()
                                if len(waypoints) >= 2:
                                    log.info(f'Scheduled run: {day_key} {time_str}')
                                    threading.Thread(
                                        target=self._fire_with_retry,
                                        args=(waypoints,),
                                        daemon=True,
                                    ).start()
                                else:
                                    log.warning(f'Scheduled run skipped — no waypoints ({len(waypoints)})')
            except Exception as e:
                log.error(f'Scheduler loop error: {e}', exc_info=True)
            time.sleep(30)

    def _fire_with_retry(self, waypoints, max_retries=10, interval=60):
        for attempt in range(max_retries + 1):
            if attempt > 0:
                log.info(f'Retry {attempt}/{max_retries} in {interval}s …')
                time.sleep(interval)
            if self._start_cb(waypoints):
                log.info('Autonomous started successfully')
                return
            log.warning(f'Start attempt {attempt + 1} failed')
        log.error(f'All {max_retries + 1} attempts failed, giving up')