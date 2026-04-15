#!/usr/bin/env python3
"""
Safety Interceptor Node for Scout Robot — Semantic + Geometric layered safety.

Two-layer architecture
======================
Layer 1  SEMANTIC  (agro_nav.policy.SemanticReflexController)

Layer 2  GEOMETRIC  (LiDAR point-cloud bounding-box zones)

Failsafe
  • If no LiDAR scan is received for LIDAR_TIMEOUT_S seconds the node publishes
    zero velocity until sensing resumes (cable pull, driver crash).

Topic mapping
  /livox/lidar              (livox_ros_driver2/CustomMsg)  LiDAR input
  /semantic_observations    (std_msgs/String, JSON)        YOLO semantic input
  /cmd_vel_raw              (geometry_msgs/Twist)          raw command input
  /cmd_vel                  (geometry_msgs/Twist)          safe command output
  /agro_nav/decision        (std_msgs/String, JSON)        debug: current decision

IMPORTANT — control_config.yaml must set:
  ros2:
    cmd_vel_topic: '/cmd_vel_raw'
so that the control panel publishes to /cmd_vel_raw, which this node intercepts
and republishes (filtered) on /cmd_vel for the Scout base driver.
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from typing import List, Optional

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
from std_msgs.msg import String

# ── agro_nav package — lives next to this file ───────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from agro_nav.policy import SemanticReflexController  # noqa: E402
from agro_nav.types import ControlCommand, SemanticObservation  # noqa: E402

# ── ANSI colours ──────────────────────────────────────────────────────────────
RESET       = "\033[0m"
BOLD_RED    = "\033[1;31m"
BOLD_YELLOW = "\033[1;33m"
BOLD_CYAN   = "\033[1;36m"
BOLD_GRAY   = "\033[1;90m"

# ── Robot geometry (Scout) ────────────────────────────────────────────────────
ROBOT_HALF_X = 0.465   # m  (half of 0.93 m body length)
ROBOT_HALF_Y = 0.350   # m  (half of 0.70 m body width)

# ── Self-filter: chassis returns to exclude from LiDAR ───────────────────────
SELF_X = 0.30          # m
SELF_Y = 0.20          # m

# ── Height band for obstacle detection ───────────────────────────────────────
Z_MIN = 0.05           # m  ignore ground returns
Z_MAX = 0.60           # m  ignore returns above the robot torso band

# ── Geometric zone margins ────────────────────────────────────────────────────
DANGER_MARGIN  = 0.15  # m  expansion beyond robot half-size → danger zone edge
TENSION_MARGIN = 0.20  # m  additional expansion → caution zone edge

DANGER_HX  = ROBOT_HALF_X + DANGER_MARGIN   # 0.615 m
DANGER_HY  = ROBOT_HALF_Y + DANGER_MARGIN   # 0.500 m
TENSION_HX = DANGER_HX + TENSION_MARGIN     # 0.815 m
TENSION_HY = DANGER_HY + TENSION_MARGIN     # 0.700 m

DANGER_THRESHOLD = 10   # minimum LiDAR points to declare a geometric danger
TENSION_CAP      = 0.1  # m/s  maximum speed inside the caution zone

# ── Timing ────────────────────────────────────────────────────────────────────
LIDAR_TIMEOUT_S    = 1.0   # s  LiDAR age threshold for failsafe full stop
SEMANTIC_TIMEOUT_S = 0.5   # s  YOLO payload age threshold before LiDAR fallback

# ── Semantic policy ───────────────────────────────────────────────────────────
# Set to robot.max_linear_speed so the GO command never artificially caps speed.
# Per-class speed scaling in agro_nav/policy.py still applies near obstacles.
NOMINAL_SPEED     = 1.5    # m/s  matches robot.max_linear_speed in control_config.yaml
FRONTAL_FOV_DEG   = 100.0  # deg  half-angle filter for semantic relevance
RECOVERY_CYCLES   = 4      # control cycles for graduated ramp after path clears
MAX_STEER_CONTRIB = 0.40   # rad/s  ceiling on steer-away angular addition

# ── LiDAR pseudo-obs (fallback when YOLO is stale) ───────────────────────────
PSEUDO_MAX_RANGE_M = 5.0   # m  frontal points beyond this are irrelevant here
PSEUDO_BIN_DEG     = 5.0   # deg  angular bin width for observation clustering
PSEUDO_MIN_POINTS_PER_BIN = 6  # minimum clustered returns before we trust a pseudo-obs
PSEUDO_IN_PATH_BEARING_DEG = 25.0  # narrower path band for LiDAR-only pseudo semantics

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class SafetyInterceptor(Node):
    """
    Intercepts /cmd_vel_raw, applies two safety layers, republishes on /cmd_vel.

    Layer 1 (semantic) modulates speed and heading based on *what* is detected.
    Layer 2 (geometric) is an unconditional hard stop based on raw LiDAR proximity.
    Either layer can independently stop the robot; neither can allow motion the
    other has forbidden.
    """

    def __init__(self) -> None:
        super().__init__("safety_interceptor")

        self._lock = threading.Lock()

        # ── Geometric state ───────────────────────────────────────────────────
        self._danger_front  = False
        self._danger_rear   = False
        self._caution_front = False
        self._caution_rear  = False
        self._caution_left  = False
        self._caution_right = False
        self._front_count   = 0
        self._rear_count    = 0
        self._last_lidar_t  = 0.0                       # time.monotonic() of last scan
        self._frontal_pts: Optional[np.ndarray] = None  # saved for pseudo-obs fallback

        # ── Semantic state ────────────────────────────────────────────────────
        self._semantics: List[SemanticObservation] = []
        self._semantics_t = 0.0

        # ── Semantic reflex controller ────────────────────────────────────────
        # Protected by a separate lock because decide() mutates internal stage state.
        self._reflex = SemanticReflexController(
            nominal_linear_speed=NOMINAL_SPEED,
            frontal_fov_deg=FRONTAL_FOV_DEG,
            recovery_cycles=RECOVERY_CYCLES,
            max_turn_rate=MAX_STEER_CONTRIB,
        )
        self._reflex_lock = threading.Lock()

        cg = ReentrantCallbackGroup()

        self.sub_lidar = self.create_subscription(
            CustomMsg, "/livox/lidar", self._lidar_cb, SENSOR_QOS,
            callback_group=cg,
        )
        self.sub_semantics = self.create_subscription(
            String, "/semantic_observations", self._semantics_cb, 20,
            callback_group=cg,
        )
        self.sub_cmd = self.create_subscription(
            Twist, "/cmd_vel_raw", self._cmd_cb, 10,
            callback_group=cg,
        )

        self.pub_cmd      = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_decision = self.create_publisher(String, "/agro_nav/decision", 20)

        self.get_logger().info(
            "SafetyInterceptor ready — semantic + geometric layers active.\n"
            "  Subscribing: /cmd_vel_raw  /livox/lidar  /semantic_observations\n"
            "  Publishing:  /cmd_vel  /agro_nav/decision"
        )

    # ── LiDAR callback ────────────────────────────────────────────────────────

    def _lidar_cb(self, msg: CustomMsg) -> None:
        if not msg.points:
            return

        pts = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float32)

        # Remove chassis reflections
        keep = ~((np.abs(pts[:, 0]) < SELF_X) & (np.abs(pts[:, 1]) < SELF_Y))
        pts  = pts[keep]
        if len(pts) == 0:
            return

        # Obstacle detection height band
        keep = (pts[:, 2] > Z_MIN) & (pts[:, 2] < Z_MAX)
        pts  = pts[keep]

        now = time.monotonic()

        if len(pts) == 0:
            with self._lock:
                self._danger_front  = self._danger_rear   = False
                self._caution_front = self._caution_rear  = False
                self._caution_left  = self._caution_right = False
                self._last_lidar_t  = now
                self._frontal_pts   = None
            return

        x  = pts[:, 0]
        y  = pts[:, 1]
        ay = np.abs(y)
        ax = np.abs(x)

        in_y_danger = ay < DANGER_HY
        in_x_danger = ax < DANGER_HX

        # Directional danger zones
        front_mask = (x > 0) & (x < DANGER_HX) & in_y_danger
        rear_mask  = (x < 0) & (x > -DANGER_HX) & in_y_danger
        fc = int(front_mask.sum())
        rc = int(rear_mask.sum())

        # Caution bands (directional)
        c_front = bool(((x > DANGER_HX) & (x < TENSION_HX) & in_y_danger).any())
        c_rear  = bool(((x < -DANGER_HX) & (x > -TENSION_HX) & in_y_danger).any())
        c_right = bool(((y >  DANGER_HY) & (y <  TENSION_HY) & in_x_danger).any())
        c_left  = bool(((y < -DANGER_HY) & (y > -TENSION_HY) & in_x_danger).any())

        # Save frontal points for pseudo-obs fallback (x > 0, within max range)
        horiz   = np.hypot(x, y)
        f_mask  = (x > 0) & (horiz < PSEUDO_MAX_RANGE_M)
        frontal = pts[f_mask]

        with self._lock:
            self._danger_front  = fc >= DANGER_THRESHOLD
            self._danger_rear   = rc >= DANGER_THRESHOLD
            self._caution_front = c_front
            self._caution_rear  = c_rear
            self._caution_left  = c_left
            self._caution_right = c_right
            self._front_count   = fc
            self._rear_count    = rc
            self._last_lidar_t  = now
            self._frontal_pts   = frontal if len(frontal) > 0 else None

    # ── Semantic callback ─────────────────────────────────────────────────────

    def _semantics_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        raw = payload if isinstance(payload, list) else payload.get("observations", [])
        parsed: List[SemanticObservation] = []
        for item in raw:
            try:
                parsed.append(SemanticObservation(
                    label=str(item["label"]),
                    distance=float(item["distance"]),
                    bearing_deg=float(item.get("bearing_deg", 0.0)),
                    confidence=float(item.get("confidence", 1.0)),
                    in_path=bool(item.get("in_path", True)),
                    relative_speed=float(item.get("relative_speed", 0.0)),
                    source=str(item.get("source", "camera")),
                    support_density=float(item.get("support_density", 0.0)),
                    depth_valid_ratio=float(item.get("depth_valid_ratio", 0.0)),
                    bbox_width_ratio=float(item.get("bbox_width_ratio", 0.0)),
                    bbox_height_ratio=float(item.get("bbox_height_ratio", 0.0)),
                    lidar_support_points=int(item.get("lidar_support_points", 0)),
                    structure_span_deg=float(item.get("structure_span_deg", 0.0)),
                ))
            except (KeyError, TypeError, ValueError):
                continue

        with self._lock:
            self._semantics   = parsed
            self._semantics_t = time.monotonic()

    # ── Command callback ──────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist) -> None:
        now = time.monotonic()

        with self._lock:
            lidar_age     = (now - self._last_lidar_t) if self._last_lidar_t else float("inf")
            sem_age       = (now - self._semantics_t)  if self._semantics_t  else float("inf")
            danger_front  = self._danger_front
            danger_rear   = self._danger_rear
            caution_front = self._caution_front
            caution_rear  = self._caution_rear
            caution_left  = self._caution_left
            caution_right = self._caution_right
            front_count   = self._front_count
            rear_count    = self._rear_count
            semantics     = list(self._semantics)
            frontal_pts   = self._frontal_pts

        lv = msg.linear.x
        av = msg.angular.z
        out = Twist()
        log_parts: list[str] = []

        # ── Failsafe: LiDAR data stale → full stop ────────────────────────────
        # Covers sensor disconnection, driver crash, or startup before first scan.
        if lidar_age > LIDAR_TIMEOUT_S:
            out.linear.x  = 0.0
            out.angular.z = 0.0
            self.get_logger().error(
                f"{BOLD_RED}[FAILSAFE] LiDAR stale ({lidar_age:.1f}s) — FULL STOP{RESET}"
            )
            self.pub_cmd.publish(out)
            self.pub_decision.publish(String(data=json.dumps({
                "action":          "STOP",
                "stage":           "FAILSAFE",
                "reason":          f"lidar_stale_{lidar_age:.1f}s",
                "dominant_label":  None,
                "threat_distance": None,
                "threat_score":    1.0,
                "sem_source":      "none",
                "danger_front":    False,
                "danger_rear":     False,
                "lv_out":          0.0,
                "av_out":          0.0,
            })))
            return

        # ── Layer 1: Semantic modulation ──────────────────────────────────────
        # Source priority: fresh YOLO > LiDAR pseudo-obs > none (pass through to geo).
        if sem_age <= SEMANTIC_TIMEOUT_S and semantics:
            observations = semantics
            sem_source   = "yolo"
        else:
            # No fresh YOLO data — build conservative "unknown" observations from
            # frontal LiDAR returns so the reflex controller still applies
            # distance-aware speed limiting in the no-vision case.
            observations = self._build_pseudo_observations(frontal_pts)
            sem_source   = "lidar_fallback" if observations else "none"

        with self._reflex_lock:
            sem_cmd = self._reflex.decide(observations)

        # LiDAR-only pseudo-observations are a conservative no-vision fallback.
        # They should not trigger semantic STOP/ADJUST churn on their own;
        # the geometric layer below already owns hard stopping from raw LiDAR.
        if sem_source == "lidar_fallback" and sem_cmd.dominant_label == "unknown":
            sem_cmd = ControlCommand(
                action="GO",
                stage=sem_cmd.stage,
                target_linear_x=self.nominal_linear_speed if hasattr(self, "nominal_linear_speed") else NOMINAL_SPEED,
                target_angular_z=0.0,
                stop=False,
                reason="lidar_fallback_deferred_to_geometry",
                dominant_label=None,
                threat_distance=sem_cmd.threat_distance,
                threat_score=sem_cmd.threat_score,
            )

        # Only modulate forward motion. The robot can always reverse away from
        # an obstacle; the semantic layer does not restrict backward motion.
        if lv > 0.01:
            if sem_cmd.stop:
                lv = 0.0
                av = 0.0
                log_parts.append(
                    f"{BOLD_YELLOW}[SEM STOP] "
                    f"{sem_cmd.reason}  "
                    f"label={sem_cmd.dominant_label}  "
                    f"dist={sem_cmd.threat_distance}  "
                    f"src={sem_source}{RESET}"
                )
            else:
                limited = min(lv, sem_cmd.target_linear_x)
                if limited < lv - 0.01:
                    log_parts.append(
                        f"{BOLD_CYAN}[SEM SLOW] "
                        f"{lv:.2f}→{limited:.2f} m/s  "
                        f"{sem_cmd.reason}  src={sem_source}{RESET}"
                    )
                lv = limited

                # Blend steer-away angular correction, clamped to MAX_STEER_CONTRIB.
                if abs(sem_cmd.target_angular_z) > 0.01:
                    av = float(np.clip(
                        av + sem_cmd.target_angular_z,
                        -MAX_STEER_CONTRIB,
                        MAX_STEER_CONTRIB,
                    ))

        out.linear.x  = lv
        out.angular.z = av

        # ── Layer 2: Geometric hard stops ──────────────────────────────────────
        capped_linear = capped_angular = False

        if out.linear.x > 0.01 and danger_front:
            out.linear.x = 0.0
            out.angular.z = 0.0
            log_parts.append(
                f"{BOLD_RED}[GEO DANGER FRONT] {front_count} pts — FULL STOP{RESET}"
            )
        elif out.linear.x < -0.01 and danger_rear:
            out.linear.x = 0.0
            out.angular.z = 0.0
            log_parts.append(
                f"{BOLD_RED}[GEO DANGER REAR] {rear_count} pts — FULL STOP{RESET}"
            )
        elif out.linear.x > 0.01 and caution_front:
            out.linear.x = min(out.linear.x, TENSION_CAP)
            capped_linear = True
        elif out.linear.x < -0.01 and caution_rear:
            out.linear.x = max(out.linear.x, -TENSION_CAP)
            capped_linear = True

        if out.angular.z > 0.01 and caution_left:
            out.angular.z = min(out.angular.z, TENSION_CAP)
            capped_angular = True
        elif out.angular.z < -0.01 and caution_right:
            out.angular.z = max(out.angular.z, -TENSION_CAP)
            capped_angular = True

        if capped_linear or capped_angular:
            log_parts.append(
                f"{BOLD_GRAY}[GEO CAUTION] "
                f"lv={out.linear.x:.2f} m/s  av={out.angular.z:.2f} rad/s{RESET}"
            )

        # ── Logging ───────────────────────────────────────────────────────────
        if log_parts:
            for part in log_parts:
                self.get_logger().warn(part)
        elif abs(out.linear.x) > 0.0 or abs(out.angular.z) > 0.0:
            self.get_logger().info(
                f"[PASS] lv={out.linear.x:.2f}  av={out.angular.z:.2f}  "
                f"sem={sem_cmd.action}({sem_source})"
            )

        # ── Publish ───────────────────────────────────────────────────────────
        self.pub_cmd.publish(out)
        self.pub_decision.publish(String(data=json.dumps({
            "action":          sem_cmd.action,
            "stage":           sem_cmd.stage,
            "reason":          sem_cmd.reason,
            "dominant_label":  sem_cmd.dominant_label,
            "threat_distance": sem_cmd.threat_distance,
            "threat_score":    round(sem_cmd.threat_score, 3),
            "sem_source":      sem_source,
            "danger_front":    danger_front,
            "danger_rear":     danger_rear,
            "lv_out":          round(out.linear.x, 3),
            "av_out":          round(out.angular.z, 3),
        })))

    # ── LiDAR pseudo-observation builder ──────────────────────────────────────

    def _build_pseudo_observations(
        self, pts: Optional[np.ndarray]
    ) -> List[SemanticObservation]:
        """
        Convert frontal LiDAR points to label='unknown' SemanticObservations.

        Points are binned into PSEUDO_BIN_DEG angular sectors; only the nearest
        return per sector is retained (~20 observations max regardless of density).

        The "unknown" risk profile in agro_nav/policy.py applies:
          caution_distance=3.0 m, stop_distance=1.0 m, speed_scale=0.15
        This is deliberately conservative for the no-vision fallback case, but it
        should still require real clustered support so sparse noise does not stop
        the robot by itself.
        """
        if pts is None or len(pts) == 0:
            return []

        x = pts[:, 0].astype(np.float64)
        y = pts[:, 1].astype(np.float64)
        dist = np.hypot(x, y)

        valid = (dist >= 0.15) & (dist <= PSEUDO_MAX_RANGE_M)
        if not valid.any():
            return []
        x, y, dist = x[valid], y[valid], dist[valid]

        bearings = np.degrees(np.arctan2(y, x))
        fov_mask = np.abs(bearings) <= (FRONTAL_FOV_DEG * 0.5)
        if not fov_mask.any():
            return []
        dist     = dist[fov_mask]
        bearings = bearings[fov_mask]

        bin_ids = (bearings / PSEUDO_BIN_DEG).astype(int)
        observations: List[SemanticObservation] = []
        for bid in np.unique(bin_ids):
            mask     = bin_ids == bid
            support_count = int(mask.sum())
            if support_count < PSEUDO_MIN_POINTS_PER_BIN:
                continue
            sub_dist = dist[mask]
            sub_bear = bearings[mask]
            idx = int(np.argmin(sub_dist))
            d   = float(sub_dist[idx])
            b   = float(sub_bear[idx])
            observations.append(SemanticObservation(
                label="unknown",
                distance=d,
                bearing_deg=b,
                confidence=min(0.8, 0.35 + 0.05 * support_count),
                in_path=abs(b) < PSEUDO_IN_PATH_BEARING_DEG,
                source="lidar_fallback",
                lidar_support_points=support_count,
            ))
        return observations


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyInterceptor()
    executor = MultiThreadedExecutor(num_threads=3)
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
