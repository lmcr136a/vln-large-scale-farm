"""
SafetyGuard — reacts to /safety_checker zone data (see sensor/safety_checker.py).

On any red zone: immediately halts all driving (Commander.set_safety_override),
which also stops any active autonomous mission. If red is still present after
`wait_sec` seconds, backs away from the obstacle:
  - red only in front zones (front/front_left/front_right) -> reverse
  - red only in back zones  (back/back_left/back_right)    -> drive forward
  - red in both front and back, or only on a side zone      -> stay stopped
Reverts to normal driving the moment no zone reports red.
"""
import logging
import threading
import time

log = logging.getLogger(__name__)

POLL_HZ = 5.0   # matches safety_checker publish rate

FRONT_ZONES = ('front', 'front_left', 'front_right')
BACK_ZONES  = ('back', 'back_left', 'back_right')


class SafetyGuard:
    def __init__(self, get_status, commander, recover_speed: float,
                 wait_sec: float = 10.0, notify=None):
        self._get_status    = get_status    # () -> {zone: 'red'|'yellow'|'green'|None}
        self._commander     = commander
        self._recover_speed = abs(recover_speed)
        self._wait_sec      = wait_sec
        self._notify        = notify        # optional fn(str) for UI status messages

        self._held       = False
        self._stop_time  = 0.0
        self._last_msg   = None

    def start(self):
        threading.Thread(target=self._loop, daemon=True, name='safety-guard').start()
        log.info(f'SafetyGuard started — wait_sec={self._wait_sec} recover_speed={self._recover_speed}')

    def _loop(self):
        interval = 1.0 / POLL_HZ
        while True:
            time.sleep(interval)
            try:
                self._tick()
            except Exception as e:
                log.error(f'SafetyGuard tick error: {e}')

    def _tick(self):
        status    = self._get_status() or {}
        any_red   = any(v == 'red' for v in status.values())
        front_red = any(status.get(z) == 'red' for z in FRONT_ZONES)
        back_red  = any(status.get(z) == 'red' for z in BACK_ZONES)

        if not any_red:
            if self._held:
                self._commander.clear_safety_override()
                self._held = False
                self._announce('Safety clear — resuming normal driving')
            return

        if not self._held:
            self._held      = True
            self._stop_time = time.time()
            self._commander.set_safety_override(0.0)
            self._announce('Obstacle detected (red zone) — stopping')
            return

        if time.time() - self._stop_time < self._wait_sec:
            self._commander.set_safety_override(0.0)
            return

        if front_red and back_red:
            self._commander.set_safety_override(0.0)
            self._announce('Obstacle on both sides — holding position')
        elif front_red:
            self._commander.set_safety_override(-self._recover_speed)
            self._announce('Obstacle ahead — reversing')
        elif back_red:
            self._commander.set_safety_override(self._recover_speed)
            self._announce('Obstacle behind — moving forward')
        else:
            self._commander.set_safety_override(0.0)
            self._announce('Obstacle to the side — holding position')

    def _announce(self, msg: str):
        if msg == self._last_msg:
            return
        self._last_msg = msg
        log.warning(f'[SafetyGuard] {msg}')
        if self._notify:
            try:
                self._notify(msg)
            except Exception:
                pass
