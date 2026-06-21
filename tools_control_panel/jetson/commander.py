import threading
import time
import logging
import yaml
import os

from geometry_msgs.msg import Twist

log = logging.getLogger(__name__)

KB_DECAY   = 0.5    # seconds — stop if no manual command arrives within this window
PUBLISH_HZ = 20.0   # steady cmd_vel rate for manual driving (decoupled from link jitter)


class Commander:
    """
    Priority: safety override > estop > manual velocity > autonomous (continue/new_path).
    Thread-safe. Used by both radio and internet command handlers.

    Manual velocity is republished at PUBLISH_HZ from a held setpoint, so the base
    keeps moving even when command packets arrive slowly or unevenly over the link
    (matching how AutonomousController drives). A watchdog zeroes the setpoint if no
    command arrives within KB_DECAY.
    """

    def __init__(self, cmd_vel_pub, auto_controller, config_path: str):
        self._pub = cmd_vel_pub
        self._auto = auto_controller
        self._cfg_path = os.path.expanduser(config_path)
        self._lock = threading.Lock()
        self._estopped = False
        self._manual = False
        self._last_kb = 0.0
        self._vx = 0.0
        self._vz = 0.0
        self._last_estop_warn = 0.0
        self._safety_vt = None   # None = no override; else forced linear.x (SafetyGuard)
        self._safety_vr = 0.0    # forced angular.z while override active

        threading.Thread(target=self._control_loop, daemon=True).start()

    # ── Safety override (highest priority — see sensor/safety_guard.py) ───────

    def set_safety_override(self, vt: float, vr: float = 0.0):
        """Force cmd_vel to (vt, vr), pre-empting estop/manual/autonomous.

        On the first call after the override was clear, *pauses* any active
        autonomous mission (it is NOT stopped) so that once the obstacle clears
        the robot resumes path following on its own. A non-zero vt/vr lets
        SafetyGuard back the robot away from a close object (e.g. reverse the
        last motion: was turning left → turn right back).
        """
        with self._lock:
            first = self._safety_vt is None
            self._safety_vt = vt
            self._safety_vr = vr
            self._manual = False
        if first:
            self._auto.pause()
            log.warning("Safety override engaged — autonomous paused, backing away if needed")

    def clear_safety_override(self):
        with self._lock:
            if self._safety_vt is None:
                return
            self._safety_vt = None
            self._safety_vr = 0.0
        self._auto.resume()
        log.info("Safety override cleared — autonomous resumed")

    def is_safety_overridden(self) -> bool:
        with self._lock:
            return self._safety_vt is not None

    def handle(self, cmd: dict):
        ctype = cmd.get("cmd")
        with self._lock:
            if self._safety_vt is not None and ctype in ("velocity", "continue", "new_path"):
                log.warning(f"'{ctype}' ignored — safety override active")
                return
            if ctype == "stop_auto":
                # Soft stop — interrupt path following, no estop flag set
                self._manual = False
                self._vx = self._vz = 0.0
                self._auto.stop()
                self._zero()
                log.info("Autonomous stopped (soft)")

            elif ctype == "estop":
                self._estopped = True
                self._manual = False
                self._vx = self._vz = 0.0
                self._auto.stop()
                self._zero()
                log.warning("EMERGENCY STOP received")

            elif ctype == "clear_estop":
                self._estopped = False
                log.info("E-STOP cleared — manual driving re-enabled")

            elif ctype == "velocity":
                if self._estopped:
                    now = time.time()
                    if now - self._last_estop_warn > 1.0:
                        self._last_estop_warn = now
                        log.warning("velocity ignored — E-STOP active (send clear_estop to resume)")
                    return
                if self._auto.is_active():
                    self._auto.stop()
                if not self._manual:
                    log.info(f"Manual driving engaged (vx={cmd.get('vx')} vz={cmd.get('vz')})")
                self._manual = True
                self._last_kb = time.time()
                self._vx = float(cmd.get("vx", 0.0))
                self._vz = float(cmd.get("vz", 0.0))
                # Published by _control_loop at PUBLISH_HZ — not here.

            elif ctype == "continue":
                self._estopped = False
                self._manual = False
                self._vx = self._vz = 0.0
                waypoints = cmd.get("waypoints")
                if waypoints:
                    resume = bool(cmd.get("resume", False))
                    self._auto.start(waypoints, resume=resume,
                                     start_index=cmd.get("start_index"))
                log.info("Resumed autonomous" if cmd.get("resume") else "Started autonomous")

            elif ctype == "new_path":
                if not self._estopped and not self._manual:
                    self._auto.start(cmd.get("waypoints", []))

            elif ctype == "config_update":
                self._apply_config(cmd.get("config", {}))

    def is_estopped(self) -> bool:
        return self._estopped

    def is_manual(self) -> bool:
        return self._manual

    def _zero(self):
        self._pub.publish(Twist())

    def _control_loop(self):
        """Publish manual velocity at a steady rate so the base never times out."""
        interval = 1.0 / PUBLISH_HZ
        while True:
            time.sleep(interval)
            twist = None
            with self._lock:
                if self._safety_vt is not None:
                    twist = Twist()                       # SafetyGuard override
                    twist.linear.x  = self._safety_vt
                    twist.angular.z = self._safety_vr
                elif self._estopped:
                    twist = Twist()                       # hold stop
                elif self._auto.is_active():
                    twist = None                          # autonomous owns cmd_vel
                elif not self._manual:
                    twist = None                          # idle
                elif time.time() - self._last_kb > KB_DECAY:
                    self._manual = False                  # watchdog: commands stopped
                    twist = Twist()
                else:
                    twist = Twist()
                    twist.linear.x  = self._vx
                    twist.angular.z = self._vz
            if twist is not None:
                self._pub.publish(twist)

    @staticmethod
    def _deep_update(base: dict, updates: dict):
        for k, v in updates.items():
            if isinstance(v, dict) and isinstance(base.get(k), dict):
                Commander._deep_update(base[k], v)
            else:
                base[k] = v

    def _apply_config(self, updates: dict):
        try:
            with open(self._cfg_path) as f:
                cfg = yaml.safe_load(f)
            self._deep_update(cfg, updates)
            with open(self._cfg_path, "w") as f:
                yaml.dump(cfg, f)
            log.info(f"Config updated: {list(updates.keys())}")
        except Exception as e:
            log.error(f"Config update failed: {e}")