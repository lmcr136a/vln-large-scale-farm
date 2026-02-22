#!/usr/bin/env python3
"""
Safety Interceptor Node for Scout Robot
Subscribes to /livox/lidar and /cmd_vel_raw, publishes to /cmd_vel.
Implements self-filtering, danger zone detection, and tension zone speed limiting.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from livox_ros_driver2.msg import CustomMsg

# ── ANSI color codes for terminal logging ──────────────────────────────────────
RESET      = "\033[0m"
BOLD_RED   = "\033[1;31m"
BOLD_GRAY  = "\033[1;90m"

# ── Robot geometry (Scout mini approximate dimensions) ─────────────────────────
ROBOT_HALF_X = 0.465   # half-length  (front/back)
ROBOT_HALF_Y = 0.350   # half-width   (left/right)

# ── Self-filter box (LiDAR sees its own chassis) ──────────────────────────────
SELF_FILTER_X = 0.30   # |x| < this  → ignored
SELF_FILTER_Y = 0.20   # |y| < this  → ignored

# ── Height band: skip ground noise and overhead clearance ─────────────────────
Z_MIN = 0.05
Z_MAX = 0.60

# ── Danger zone margins ───────────────────────────────────────────────────────
DANGER_MARGIN    = 0.15   # robot body + this  → danger box
TENSION_MARGIN   = 0.20   # danger edge + this → tension box

DANGER_HALF_X = ROBOT_HALF_X + DANGER_MARGIN          # 0.615 m
DANGER_HALF_Y = ROBOT_HALF_Y + DANGER_MARGIN          # 0.500 m
TENSION_HALF_X = DANGER_HALF_X + TENSION_MARGIN       # 0.815 m
TENSION_HALF_Y = DANGER_HALF_Y + TENSION_MARGIN       # 0.700 m

# ── Detection threshold ───────────────────────────────────────────────────────
DANGER_POINT_THRESHOLD = 10

# ── Speed cap inside tension zone ─────────────────────────────────────────────
TENSION_SPEED_CAP = 0.1


class SafetyInterceptor(Node):
    def __init__(self):
        super().__init__("safety_interceptor")

        self._collision_detected = False
        self._latest_cmd: Twist | None = None

        self.sub_lidar = self.create_subscription(
            CustomMsg, "/livox/lidar", self._lidar_cb, 10
        )
        self.sub_cmd = self.create_subscription(
            Twist, "/cmd_vel_raw", self._cmd_cb, 10
        )
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("SafetyInterceptor started.")

    # ── Helpers ────────────────────────────────────────────────────────────────

    @staticmethod
    def _extract_points(msg: CustomMsg) -> np.ndarray:
        """Convert Livox CustomMsg points to (N,3) float32 numpy array."""
        n = len(msg.points)
        if n == 0:
            return np.empty((0, 3), dtype=np.float32)
        pts = np.empty((n, 3), dtype=np.float32)
        for i, p in enumerate(msg.points):
            pts[i, 0] = p.x
            pts[i, 1] = p.y
            pts[i, 2] = p.z
        return pts

    @staticmethod
    def _self_filter(pts: np.ndarray) -> np.ndarray:
        """Remove points that fall inside the robot chassis bounding box."""
        mask = ~(
            (np.abs(pts[:, 0]) < SELF_FILTER_X) &
            (np.abs(pts[:, 1]) < SELF_FILTER_Y)
        )
        return pts[mask]

    @staticmethod
    def _height_filter(pts: np.ndarray) -> np.ndarray:
        """Keep only points within the valid height band."""
        mask = (pts[:, 2] > Z_MIN) & (pts[:, 2] < Z_MAX)
        return pts[mask]

    @staticmethod
    def _in_box(pts: np.ndarray, hx: float, hy: float) -> np.ndarray:
        """Return boolean mask for points inside an axis-aligned box ±hx, ±hy."""
        return (np.abs(pts[:, 0]) < hx) & (np.abs(pts[:, 1]) < hy)

    # ── LiDAR callback ────────────────────────────────────────────────────────

    def _lidar_cb(self, msg: CustomMsg):
        pts = self._extract_points(msg)
        if pts.shape[0] == 0:
            return

        pts = self._self_filter(pts)
        pts = self._height_filter(pts)
        if pts.shape[0] == 0:
            return

        cmd = self._latest_cmd

        # Determine current motion intent
        linear_v  = cmd.linear.x  if cmd else 0.0
        angular_v = cmd.angular.z if cmd else 0.0

        moving_forward  = linear_v  >  0.01
        moving_backward = linear_v  < -0.01
        turning         = abs(angular_v) > 0.01

        # ── Danger zone: full box, all sides ──────────────────────────────────
        danger_mask  = self._in_box(pts, DANGER_HALF_X, DANGER_HALF_Y)
        danger_count = int(danger_mask.sum())
        self._collision_detected = danger_count >= DANGER_POINT_THRESHOLD

        # ── Tension zone: directional ─────────────────────────────────────────
        in_tension = False
        if not self._collision_detected:
            if moving_forward:
                # Check front band: x in (DANGER_HALF_X, TENSION_HALF_X), |y| < DANGER_HALF_Y
                front_mask = (
                    (pts[:, 0] >  DANGER_HALF_X) &
                    (pts[:, 0] <  TENSION_HALF_X) &
                    (np.abs(pts[:, 1]) < DANGER_HALF_Y)
                )
                in_tension = bool(front_mask.any())

            elif moving_backward:
                # Check rear band
                rear_mask = (
                    (pts[:, 0] < -DANGER_HALF_X) &
                    (pts[:, 0] > -TENSION_HALF_X) &
                    (np.abs(pts[:, 1]) < DANGER_HALF_Y)
                )
                in_tension = bool(rear_mask.any())

            if turning:
                # Check lateral bands
                lat_mask = (
                    (np.abs(pts[:, 1]) > DANGER_HALF_Y) &
                    (np.abs(pts[:, 1]) < TENSION_HALF_Y) &
                    (np.abs(pts[:, 0]) < DANGER_HALF_X)
                )
                in_tension = in_tension or bool(lat_mask.any())

        # ── Build output command ───────────────────────────────────────────────
        out = Twist()

        if self._collision_detected:
            out.linear.x  = 0.0
            out.angular.z = 0.0
            self.get_logger().warn(
                f"{BOLD_RED}[DANGER ZONE] Obstacle detected ({danger_count} pts) "
                f"— EMERGENCY STOP{RESET}"
            )

        elif in_tension:
            if cmd:
                # Cap speeds to TENSION_SPEED_CAP while preserving sign/direction
                lv = cmd.linear.x
                av = cmd.angular.z
                out.linear.x  = float(np.clip(lv,  -TENSION_SPEED_CAP, TENSION_SPEED_CAP))
                out.angular.z = float(np.clip(av,  -TENSION_SPEED_CAP, TENSION_SPEED_CAP))
            if abs(out.linear.x) > 0.0 or abs(out.angular.z) > 0.0:
                self.get_logger().warn(
                    f"{BOLD_GRAY}[CAUTION ZONE] Obstacle nearby — "
                    f"linear={out.linear.x:.2f} m/s  angular={out.angular.z:.2f} rad/s{RESET}"
                )

        else:
            if cmd:
                out.linear.x  = cmd.linear.x
                out.angular.z = cmd.angular.z
            if abs(out.linear.x) > 0.0 or abs(out.angular.z) > 0.0:
                self.get_logger().info(
                    f"[PASS-THROUGH] linear={out.linear.x:.2f} m/s  "
                    f"angular={out.angular.z:.2f} rad/s"
                )

        self.pub_cmd.publish(out)

    # ── /cmd_vel_raw callback ──────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        """Cache the latest velocity command from the web app."""
        self._latest_cmd = msg


def main(args=None):
    rclpy.init(args=args)
    node = SafetyInterceptor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()