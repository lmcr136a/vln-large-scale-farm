import threading
import time
import logging
import yaml
import os

from geometry_msgs.msg import Twist

log = logging.getLogger(__name__)

KB_DECAY = 0.5   # seconds — stop if no keyboard command received


class Commander:
    """
    Priority: estop > keyboard_velocity > autonomous (continue/new_path).
    Thread-safe. Used by both radio and internet command handlers.
    """

    def __init__(self, cmd_vel_pub, auto_controller, config_path: str):
        self._pub = cmd_vel_pub
        self._auto = auto_controller
        self._cfg_path = os.path.expanduser(config_path)
        self._lock = threading.Lock()
        self._estopped = False
        self._manual = False
        self._last_kb = 0.0

        threading.Thread(target=self._kb_decay_loop, daemon=True).start()

    def handle(self, cmd: dict):
        ctype = cmd.get("cmd")
        with self._lock:
            if ctype == "stop_auto":
                # Soft stop — interrupt path following, no estop flag set
                self._manual = False
                self._auto.stop()
                self._zero()
                log.info("Autonomous stopped (soft)")

            elif ctype == "estop":
                self._estopped = True
                self._manual = False
                self._auto.stop()
                self._zero()
                log.warning("EMERGENCY STOP received")

            elif ctype == "velocity":
                if self._estopped:
                    return
                if self._auto.is_active():
                    self._auto.stop()
                self._manual = True
                self._last_kb = time.time()
                t = Twist()
                t.linear.x = float(cmd.get("vx", 0.0))
                t.angular.z = float(cmd.get("vz", 0.0))
                self._pub.publish(t)

            elif ctype == "continue":
                self._estopped = False
                self._manual = False
                waypoints = cmd.get("waypoints")
                if waypoints:
                    self._auto.start(waypoints)
                log.info("Resumed autonomous")

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

    def _kb_decay_loop(self):
        while True:
            time.sleep(0.05)
            with self._lock:
                if self._manual and time.time() - self._last_kb > KB_DECAY:
                    self._manual = False
                    self._zero()

    def _apply_config(self, updates: dict):
        try:
            with open(self._cfg_path) as f:
                cfg = yaml.safe_load(f)
            cfg.update(updates)
            with open(self._cfg_path, "w") as f:
                yaml.dump(cfg, f)
            log.info(f"Config updated: {list(updates.keys())}")
        except Exception as e:
            log.error(f"Config update failed: {e}")