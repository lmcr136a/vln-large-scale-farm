"""
SafetyGuard — reacts to /safety_checker zone data (see sensor/safety_checker.py).

On any red zone (close object), it *immediately* backs the robot away by
reversing the motion it was making — no waiting. The autonomous mission is only
paused (not stopped), so it resumes on its own once the object clears:
  - red only in front zones (front/front_left/front_right) -> reverse (go back)
  - red only in back zones  (back/back_left/back_right)     -> drive forward
  - red in both front and back, or only on a side zone       -> stay stopped
Steering is reversed too: if it was turning left, it turns right back, retracing
its path out of the red zone. The back-away motion itself is never paused — only
the forward path-following into the red zone is held.
Reverts to normal driving the moment no zone reports red.
"""
import logging
import threading
import time

log = logging.getLogger(__name__)

POLL_HZ = 5.0   # matches safety_checker publish rate

# Only the straight-ahead and straight-behind zones matter. Side zones
# ('left'/'right') AND corner zones ('front_left'/'front_right'/'back_left'/
# 'back_right') are ignored entirely — the robot only reacts to something
# directly in front or directly behind.
FRONT_ZONES = ('front',)
BACK_ZONES  = ('back',)


class SafetyGuard:
    def __init__(self, get_status, commander, recover_speed: float,
                 wait_sec: float = 10.0, notify=None, auto_ctrl=None):
        self._get_status    = get_status    # () -> {zone: 'red'|'yellow'|'green'|None}
        self._commander     = commander
        self._recover_speed = abs(recover_speed)
        self._wait_sec      = wait_sec      # kept for compatibility; back-away is immediate
        self._notify        = notify        # optional fn(str) for UI status messages
        self._auto          = auto_ctrl     # AutonomousController — to read last motion

        self._held       = False
        self._recover_vr = 0.0   # reversed steering, captured when red first appears
        self._last_msg   = None
        self._enabled    = True  # toggled from the panel; off → never intervenes

    def set_enabled(self, enabled: bool):
        """Enable/disable obstacle reaction. When disabled, any active back-away
        override is cleared and the guard stops intervening until re-enabled."""
        enabled = bool(enabled)
        if enabled == self._enabled:
            return
        self._enabled = enabled
        if not enabled and self._held:
            self._commander.clear_safety_override()
            self._held = False
        self._announce('Safety checker ENABLED' if enabled else 'Safety checker DISABLED')

    def is_enabled(self) -> bool:
        return self._enabled

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
        if not self._enabled:
            if self._held:
                self._commander.clear_safety_override()
                self._held = False
            return
        # Manual teleop has priority: while the operator is actively driving, the
        # guard steps aside (and drops any held override) so manual is never
        # blocked. It resumes protecting once manual goes idle / autonomous runs.
        if self._commander.is_manual():
            if self._held:
                self._commander.clear_safety_override()
                self._held = False
            return
        status    = self._get_status() or {}
        front_red = any(status.get(z) == 'red' for z in FRONT_ZONES)
        back_red  = any(status.get(z) == 'red' for z in BACK_ZONES)
        # Only front/back obstacles block driving. An object purely to the left or
        # right ('left'/'right' zones) must NOT stop forward/backward motion.
        any_red   = front_red or back_red

        if not any_red:
            if self._held:
                self._commander.clear_safety_override()
                self._held = False
                self._announce('Safety clear — resuming normal driving')
            return

        # Red zone present. On the first tick, capture the motion the robot was
        # making so we can reverse the turn (was turning left → turn right back).
        if not self._held:
            self._held = True
            vr_last = 0.0
            if self._auto is not None:
                try:
                    _, vr_last = self._auto.last_command()
                except Exception:
                    vr_last = 0.0
            self._recover_vr = -vr_last   # reverse the steering direction

        # Back away immediately by reversing the last motion — no "holding
        # position" state. Front obstacle takes priority: reverse out. Otherwise
        # something is behind → drive forward. (If both are red, front wins and
        # we still back off rather than freezing in place.)
        if front_red:
            self._commander.set_safety_override(-self._recover_speed, self._recover_vr)
            self._announce('Obstacle ahead — backing away')
        else:  # back_red
            self._commander.set_safety_override(self._recover_speed, self._recover_vr)
            self._announce('Obstacle behind — moving forward')

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
