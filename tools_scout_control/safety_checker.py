#!/usr/bin/env python3
"""
Safety Interceptor Node for Scout Robot — directional obstacle stop.

Danger zone is split into FRONT and REAR halves.
- Obstacle in FRONT danger zone → block forward motion only (backward still allowed)
- Obstacle in REAR  danger zone → block backward motion only (forward still allowed)
- Caution zone caps speed to TENSION_CAP in the affected direction only.

Architecture:
  - LiDAR callback  : updates shared state only (non-blocking)
  - cmd_vel callback: reads cached state, enforces limits, publishes immediately
  - MultiThreadedExecutor + ReentrantCallbackGroup → parallel execution
"""

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from livox_ros_driver2.msg import CustomMsg
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

# ── ANSI colors ────────────────────────────────────────────────────────────────
RESET     = "\033[0m"
BOLD_RED  = "\033[1;31m"
BOLD_GRAY = "\033[1;90m"

# ── Robot geometry (Scout) ─────────────────────────────────────────────────────
ROBOT_HALF_X = 0.465
ROBOT_HALF_Y = 0.350

# ── Self-filter (chassis bounding box) ────────────────────────────────────────
SELF_X = 0.30
SELF_Y = 0.20

# ── Height band ───────────────────────────────────────────────────────────────
Z_MIN = 0.05
Z_MAX = 0.60

# ── Zone margins ──────────────────────────────────────────────────────────────
DANGER_MARGIN  = 0.15
TENSION_MARGIN = 0.20

DANGER_HX  = ROBOT_HALF_X + DANGER_MARGIN   # 0.615 m
DANGER_HY  = ROBOT_HALF_Y + DANGER_MARGIN   # 0.500 m
TENSION_HX = DANGER_HX + TENSION_MARGIN     # 0.815 m
TENSION_HY = DANGER_HY + TENSION_MARGIN     # 0.700 m

DANGER_THRESHOLD = 10
TENSION_CAP      = 0.1

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class SafetyInterceptor(Node):
    def __init__(self):
        super().__init__("safety_interceptor")

        # Shared state — written by LiDAR thread, read by cmd thread
        self._lock = threading.Lock()

        # Directional danger flags
        self._danger_front   = False   # obstacle in front   danger zone
        self._danger_rear    = False   # obstacle in rear    danger zone

        # Directional caution flags
        self._caution_front  = False   # obstacle in front   caution band
        self._caution_rear   = False   # obstacle in rear    caution band
        self._caution_left   = False   # obstacle in lateral caution band
        self._caution_right  = False   # obstacle in lateral caution band

        self._front_count    = 0
        self._rear_count     = 0

        self._latest_cmd: Twist | None = None

        cg = ReentrantCallbackGroup()

        self.sub_lidar = self.create_subscription(
            CustomMsg, "/livox/lidar", self._lidar_cb, SENSOR_QOS,
            callback_group=cg,
        )
        self.sub_cmd = self.create_subscription(
            Twist, "/cmd_vel_raw", self._cmd_cb, 10,
            callback_group=cg,
        )
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("SafetyInterceptor started (directional mode).")

    # ── LiDAR callback ────────────────────────────────────────────────────────

    def _lidar_cb(self, msg: CustomMsg):
        if not msg.points:
            return

        pts = np.array(
            [[p.x, p.y, p.z] for p in msg.points], dtype=np.float32
        )

        # Self-filter
        keep = ~((np.abs(pts[:, 0]) < SELF_X) & (np.abs(pts[:, 1]) < SELF_Y))
        pts  = pts[keep]
        if pts.shape[0] == 0:
            return

        # Height filter
        keep = (pts[:, 2] > Z_MIN) & (pts[:, 2] < Z_MAX)
        pts  = pts[keep]

        # Reset all flags if no valid points remain
        if pts.shape[0] == 0:
            with self._lock:
                self._danger_front  = self._danger_rear   = False
                self._caution_front = self._caution_rear  = False
                self._caution_left  = self._caution_right = False
            return

        x  = pts[:, 0]
        ax = np.abs(x)
        ay = np.abs(pts[:, 1])

        # ── Shared masks ──────────────────────────────────────────────────────
        in_y_danger  = ay < DANGER_HY
        in_y_tension = ay < DANGER_HY          # same y-band for front/rear caution
        in_x_danger  = ax < DANGER_HX

        # ── DANGER ZONE (directional) ─────────────────────────────────────────
        # Front: x > 0, within danger box
        front_danger_mask = (x > 0) & (x < DANGER_HX) & in_y_danger
        rear_danger_mask  = (x < 0) & (x > -DANGER_HX) & in_y_danger

        front_count = int(front_danger_mask.sum())
        rear_count  = int(rear_danger_mask.sum())

        danger_front = front_count >= DANGER_THRESHOLD
        danger_rear  = rear_count  >= DANGER_THRESHOLD

        # ── CAUTION ZONE (directional) ────────────────────────────────────────
        # Front band: x in (DANGER_HX, TENSION_HX), within danger y
        caution_front = bool(
            ((x > DANGER_HX) & (x < TENSION_HX) & in_y_tension).any()
        )
        # Rear band
        caution_rear  = bool(
            ((x < -DANGER_HX) & (x > -TENSION_HX) & in_y_tension).any()
        )
        # Lateral bands (both left and right, for turning)
        caution_right = bool(
            ((pts[:, 1] >  DANGER_HY) & (pts[:, 1] <  TENSION_HY) & in_x_danger).any()
        )
        caution_left  = bool(
            ((pts[:, 1] < -DANGER_HY) & (pts[:, 1] > -TENSION_HY) & in_x_danger).any()
        )

        with self._lock:
            self._danger_front  = danger_front
            self._danger_rear   = danger_rear
            self._caution_front = caution_front
            self._caution_rear  = caution_rear
            self._caution_left  = caution_left
            self._caution_right = caution_right
            self._front_count   = front_count
            self._rear_count    = rear_count

    # ── cmd_vel callback ───────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        self._latest_cmd = msg

        with self._lock:
            danger_front  = self._danger_front
            danger_rear   = self._danger_rear
            caution_front = self._caution_front
            caution_rear  = self._caution_rear
            caution_left  = self._caution_left
            caution_right = self._caution_right
            front_count   = self._front_count
            rear_count    = self._rear_count

        lv  = msg.linear.x
        av  = msg.angular.z
        out = Twist()
        out.linear.x  = lv
        out.angular.z = av

        blocked_linear  = False
        capped_linear   = False
        capped_angular  = False
        log_parts       = []

        # ── Linear velocity: direction-aware danger check ─────────────────────
        if lv > 0.01 and danger_front:
            # Trying to go forward but obstacle ahead → block forward only
            out.linear.x = 0.0
            blocked_linear = True
            log_parts.append(
                f"{BOLD_RED}[DANGER FRONT] {front_count} pts — forward BLOCKED{RESET}"
            )

        elif lv < -0.01 and danger_rear:
            # Trying to go backward but obstacle behind → block backward only
            out.linear.x = 0.0
            blocked_linear = True
            log_parts.append(
                f"{BOLD_RED}[DANGER REAR] {rear_count} pts — backward BLOCKED{RESET}"
            )

        # ── Linear velocity: direction-aware caution cap ──────────────────────
        elif lv > 0.01 and caution_front:
            out.linear.x = min(lv, TENSION_CAP)
            capped_linear = True

        elif lv < -0.01 and caution_rear:
            out.linear.x = max(lv, -TENSION_CAP)
            capped_linear = True

        # ── Angular velocity: caution cap based on turning direction ──────────
        # Positive angular.z = turning left (CCW)
        # → left side becomes the swept side → check caution_left
        if av > 0.01 and caution_left:
            out.angular.z = min(av, TENSION_CAP)
            capped_angular = True
        elif av < -0.01 and caution_right:
            out.angular.z = max(av, -TENSION_CAP)
            capped_angular = True

        # ── Logging ───────────────────────────────────────────────────────────
        moving = abs(out.linear.x) > 0.0 or abs(out.angular.z) > 0.0

        if capped_linear or capped_angular:
            log_parts.append(
                f"{BOLD_GRAY}[CAUTION] "
                f"linear={out.linear.x:.2f} m/s  "
                f"angular={out.angular.z:.2f} rad/s{RESET}"
            )

        if log_parts:
            for part in log_parts:
                self.get_logger().warn(part)
        elif moving:
            self.get_logger().info(
                f"[PASS] linear={out.linear.x:.2f} m/s  "
                f"angular={out.angular.z:.2f} rad/s"
            )

        self.pub_cmd.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyInterceptor()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()