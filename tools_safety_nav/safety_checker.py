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
from collections import deque
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
from agro_nav.types import BoundingBox2D, ControlCommand, SemanticObservation  # noqa: E402

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

DANGER_THRESHOLD       = 10   # minimum *hard* LiDAR points to declare a geometric danger
DANGER_CLEAR_THRESHOLD = 5    # hysteresis: un-declare danger below this count
SOFT_WEIGHT            = 0.25 # fractional contribution of soft returns toward danger count;
                               # 40 soft stems → 10 effective → same threshold as 10 hard
                               # Livox MID360 empirical: vegetation/soil reflectivity ≈ 10–60,
                               # metal/wood/paint ≈ 80–255
HARD_REFLECTIVITY      = 70   # reflectivity threshold separating hard from soft returns
DANGER_EMA_ALPHA       = 0.40 # EMA smoothing for danger count; ~3 scans to settle
                               # prevents a single wind-gust scan from triggering a stop
TENSION_CAP            = 0.1  # m/s  maximum speed inside the caution zone (rigid obstacles)
TENSION_CAP_COMPLIANT  = 0.4  # m/s  caution-zone cap when compliant vegetation confirmed;
                               # allows the robot to push through crop rows at a safe crawl

# ── Timing ────────────────────────────────────────────────────────────────────
LIDAR_TIMEOUT_S    = 1.0   # s  LiDAR age threshold for failsafe full stop
SEMANTIC_TIMEOUT_S = 0.8   # s  YOLO payload age threshold before LiDAR fallback;
                           # short persistence avoids semantic on/off flicker between frames
CMD_TIMEOUT_S      = 0.35  # s  stale /cmd_vel_raw age before commanded motion is cleared

# ── Semantic policy ───────────────────────────────────────────────────────────
# Set to robot.max_linear_speed so the GO command never artificially caps speed.
# Per-class speed scaling in agro_nav/policy.py still applies near obstacles.
NOMINAL_SPEED     = 1.5    # m/s  matches robot.max_linear_speed in control_config.yaml
FRONTAL_FOV_DEG   = 100.0  # deg  half-angle filter for semantic relevance
RECOVERY_CYCLES   = 4      # control cycles for graduated ramp after path clears
MAX_STEER_CONTRIB = 0.40   # rad/s  ceiling on steer-away angular addition

# ── Gap-finding fallback (VFH-style) ─────────────────────────────────────────
# After GAP_STEER_CYCLES consecutive full-stop command cycles (robot wanted to
# move but both layers zeroed it), the polar-histogram gap finder fires.  It bins
# the frontal LiDAR cloud, finds the most open direction within ±60°, and steers
# toward it.  This replaces the ADJUST mechanism's arbitrary left/right alternation
# with an evidence-based direction drawn from the actual obstacle geometry.
GAP_STEER_CYCLES     = 8     # ~0.8 s at 10 Hz autonomous command rate
GAP_STEER_RATE       = 0.35  # rad/s — conservative enough to not overshoot a gap
GAP_BIN_DEG          = 12.0  # polar-histogram bin width
GAP_MIN_CLEAR_M      = 1.5   # a "gap" must be at least this deep to be worth turning toward
GAP_MIN_BEARING_DEG  = 15.0  # gap must be at least this far off-centre to be actionable

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
        self._cycle_lock = threading.Lock()

        # ── Gap-steer state ───────────────────────────────────────────────────
        self._consecutive_stops  = 0    # cycles where input wanted forward but output was zero
        self._gap_bearing_history: "deque[Optional[float]]" = deque(maxlen=3)

        # ── Geometric state ───────────────────────────────────────────────────
        self._front_ema     = 0.0   # EMA of effective front danger count
        self._rear_ema      = 0.0   # EMA of effective rear  danger count
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
        self._latest_cmd = Twist()
        self._latest_cmd_t = 0.0
        self._last_log_signature: Optional[str] = None
        self._last_log_time = 0.0

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
        self.create_timer(0.1, self._control_cycle, callback_group=cg)

        self.get_logger().info(
            "SafetyInterceptor ready — semantic + geometric layers active.\n"
            "  Subscribing: /cmd_vel_raw  /livox/lidar  /semantic_observations\n"
            "  Publishing:  /cmd_vel  /agro_nav/decision"
        )

    def _select_primary_observation(
        self, observations: List[SemanticObservation], sem_source: str
    ) -> Optional[SemanticObservation]:
        if not observations:
            return None
        candidates = observations
        if sem_source == "yolo":
            camera_candidates = [item for item in observations if item.source != "lidar_fallback"]
            if camera_candidates:
                candidates = camera_candidates

        return min(
            candidates,
            key=lambda item: (
                0 if item.in_path else 1,
                item.distance,
                -item.confidence,
                abs(item.bearing_deg),
            ),
        )

    def _classify_observation_compliance(
        self, observation: Optional[SemanticObservation]
    ) -> str:
        if observation is None:
            return "none"
        profile = self._reflex.risk_profiles.get(
            observation.label,
            self._reflex.risk_profiles["unknown"],
        )
        if profile.family == "compliant":
            return "compliant" if self._reflex._passes_compliance_check(observation) else "non_compliant"
        if profile.family == "semi_compliant":
            return "semi_compliant"
        return "non_compliant"

    def _should_force_vulnerable_stop(
        self,
        observation: Optional[SemanticObservation],
        sem_source: str,
    ) -> bool:
        if observation is None or sem_source != "yolo":
            return False

        profile = self._reflex.risk_profiles.get(
            observation.label,
            self._reflex.risk_profiles["unknown"],
        )
        if profile.family != "vulnerable":
            return False
        if not observation.in_path:
            return False
        if abs(observation.bearing_deg) > 28.0:
            return False
        if observation.confidence < max(self._reflex.min_confidence, 0.30):
            return False

        close_in_distance = observation.distance <= max(profile.stop_distance + 0.50, 2.20)
        large_in_frame = (
            observation.bbox_height_ratio >= 0.22
            or observation.bbox_width_ratio >= 0.16
        )
        return close_in_distance or large_in_frame

    # ── LiDAR callback ────────────────────────────────────────────────────────

    def _lidar_cb(self, msg: CustomMsg) -> None:
        if not msg.points:
            return

        pts  = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float32)
        # Reflectivity per point: Livox MID360 uint8, vegetation ≈ 10–60, hard surfaces ≈ 80–255.
        # Used to separate soft (compliant) returns from hard (rigid) returns so that crop
        # stems and leaf canopy do not count as strongly toward the danger threshold.
        try:
            refl = np.array([p.reflectivity for p in msg.points], dtype=np.float32)
        except AttributeError:
            refl = np.full(len(pts), 128.0, dtype=np.float32)  # safe fallback: treat as hard

        # Remove chassis reflections
        keep = ~((np.abs(pts[:, 0]) < SELF_X) & (np.abs(pts[:, 1]) < SELF_Y))
        pts  = pts[keep]
        refl = refl[keep]
        if len(pts) == 0:
            return

        # Obstacle detection height band
        keep = (pts[:, 2] > Z_MIN) & (pts[:, 2] < Z_MAX)
        pts  = pts[keep]
        refl = refl[keep]

        now = time.monotonic()

        if len(pts) == 0:
            with self._lock:
                self._danger_front  = self._danger_rear   = False
                self._caution_front = self._caution_rear  = False
                self._caution_left  = self._caution_right = False
                self._last_lidar_t  = now
                self._frontal_pts   = None
            return

        x    = pts[:, 0]
        y    = pts[:, 1]
        ay   = np.abs(y)
        ax   = np.abs(x)
        hard = refl >= HARD_REFLECTIVITY  # True = rigid surface

        in_y_danger = ay < DANGER_HY
        in_x_danger = ax < DANGER_HX

        # Directional danger zones — weighted by material hardness.
        # Hard returns (metal, wood, painted surfaces) count fully.
        # Soft returns (vegetation, soil) count at SOFT_WEIGHT so that dense crop
        # stems don't trigger a stop unless there are an unreasonably large number.
        front_mask = (x > 0) & (x < DANGER_HX) & in_y_danger
        rear_mask  = (x < 0) & (x > -DANGER_HX) & in_y_danger

        fc_eff = float(front_mask[hard].sum()) + SOFT_WEIGHT * float(front_mask[~hard].sum())
        rc_eff = float(rear_mask[hard].sum())  + SOFT_WEIGHT * float(rear_mask[~hard].sum())

        # EMA smoothing: prevents a single wind-gust scan from flipping the danger state.
        # We update the EMA here (called at LiDAR frequency ~10 Hz) and use the smoothed
        # value for threshold comparisons.
        with self._lock:
            self._front_ema = DANGER_EMA_ALPHA * fc_eff + (1.0 - DANGER_EMA_ALPHA) * self._front_ema
            self._rear_ema  = DANGER_EMA_ALPHA * rc_eff + (1.0 - DANGER_EMA_ALPHA) * self._rear_ema
            fc_smooth = self._front_ema
            rc_smooth = self._rear_ema

        fc = int(round(fc_smooth))
        rc = int(round(rc_smooth))

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
            # Hysteresis: declare danger at DANGER_THRESHOLD, only clear below
            # DANGER_CLEAR_THRESHOLD.  Prevents LiDAR boundary flicker (9 pts vs 11 pts)
            # from causing rapid stop/go oscillation.
            if fc >= DANGER_THRESHOLD:
                self._danger_front = True
            elif fc < DANGER_CLEAR_THRESHOLD:
                self._danger_front = False
            # else: keep previous state

            if rc >= DANGER_THRESHOLD:
                self._danger_rear = True
            elif rc < DANGER_CLEAR_THRESHOLD:
                self._danger_rear = False
            # else: keep previous state

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
                bbox_payload = item.get("bbox") or {}
                bbox = None
                if {"x_min", "y_min", "x_max", "y_max"} <= set(bbox_payload):
                    bbox = BoundingBox2D(
                        x_min=float(bbox_payload["x_min"]),
                        y_min=float(bbox_payload["y_min"]),
                        x_max=float(bbox_payload["x_max"]),
                        y_max=float(bbox_payload["y_max"]),
                    )
                parsed.append(SemanticObservation(
                    label=str(item["label"]),
                    distance=float(item["distance"]),
                    bearing_deg=float(item.get("bearing_deg", 0.0)),
                    confidence=float(item.get("confidence", 1.0)),
                    in_path=bool(item.get("in_path", True)),
                    relative_speed=float(item.get("relative_speed", 0.0)),
                    source=str(item.get("source", "camera")),
                    bbox=bbox,
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
        with self._lock:
            self._latest_cmd.linear.x = msg.linear.x
            self._latest_cmd.linear.y = msg.linear.y
            self._latest_cmd.linear.z = msg.linear.z
            self._latest_cmd.angular.x = msg.angular.x
            self._latest_cmd.angular.y = msg.angular.y
            self._latest_cmd.angular.z = msg.angular.z
            self._latest_cmd_t = time.monotonic()
        self._control_cycle()

    def _throttled_log(
        self,
        log_parts: List[str],
        out: Twist,
        sem_cmd: ControlCommand,
        sem_source: str,
        cmd_active: bool,
    ) -> None:
        now = time.monotonic()
        if log_parts:
            signature = "warn|" + "|".join(log_parts)
            if signature == self._last_log_signature and (now - self._last_log_time) < 0.75:
                return
            self._last_log_signature = signature
            self._last_log_time = now
            for part in log_parts:
                self.get_logger().warn(part)
            return

        if not cmd_active:
            return
        if abs(out.linear.x) <= 0.0 and abs(out.angular.z) <= 0.0:
            return

        signature = (
            f"info|{round(out.linear.x, 2)}|{round(out.angular.z, 2)}|"
            f"{sem_cmd.action}|{sem_source}"
        )
        if signature == self._last_log_signature and (now - self._last_log_time) < 0.75:
            return
        self._last_log_signature = signature
        self._last_log_time = now
        self.get_logger().info(
            f"[PASS] lv={out.linear.x:.2f}  av={out.angular.z:.2f}  "
            f"sem={sem_cmd.action}({sem_source})"
        )

    def _control_cycle(self) -> None:
        if not self._cycle_lock.acquire(blocking=False):
            return
        try:
            now = time.monotonic()

            with self._lock:
                lidar_age     = (now - self._last_lidar_t) if self._last_lidar_t else float("inf")
                sem_age       = (now - self._semantics_t)  if self._semantics_t  else float("inf")
                cmd_age       = (now - self._latest_cmd_t) if self._latest_cmd_t else float("inf")
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
                cmd_linear_x  = self._latest_cmd.linear.x
                cmd_angular_z = self._latest_cmd.angular.z

            cmd_active = cmd_age <= CMD_TIMEOUT_S
            lv    = cmd_linear_x if cmd_active else 0.0
            av    = cmd_angular_z if cmd_active else 0.0
            lv_in = lv  # remember original forward intent before any layer modifies it
            av_in = av
            out = Twist()
            log_parts: list[str] = []
            semantic_status = "offline"
            if sem_age != float("inf"):
                semantic_status = "live" if sem_age <= SEMANTIC_TIMEOUT_S else "stale"
            semantic_detection_count = len(semantics)

            # ── Failsafe: LiDAR data stale → full stop ────────────────────────────
            # Covers sensor disconnection, driver crash, or startup before first scan.
            if lidar_age > LIDAR_TIMEOUT_S:
                out.linear.x  = 0.0
                out.angular.z = 0.0
                if np.isfinite(lidar_age):
                    signature = f"failsafe|{int(lidar_age * 10)}"
                else:
                    signature = "failsafe|inf"
                if signature != self._last_log_signature or (now - self._last_log_time) >= 0.75:
                    self._last_log_signature = signature
                    self._last_log_time = now
                    self.get_logger().error(
                        f"{BOLD_RED}[FAILSAFE] LiDAR stale ({lidar_age:.1f}s) — FULL STOP{RESET}"
                    )
                self.pub_cmd.publish(out)
                self.pub_decision.publish(String(data=json.dumps({
                    "action":          "STOP",
                    "stage":           "FAILSAFE",
                    "reason":          f"lidar_stale_{lidar_age:.1f}s",
                    "semantic_action": None,
                    "semantic_stage":  None,
                    "semantic_reason": None,
                    "semantic_status": semantic_status,
                    "semantic_detection_count": semantic_detection_count,
                    "decision_layer":  "failsafe",
                    "dominant_label":  None,
                    "compliance":      "none",
                    "primary_semantic_label": None,
                    "primary_semantic_distance": None,
                    "primary_semantic_confidence": None,
                    "primary_semantic_in_path": None,
                    "primary_semantic_source": None,
                    "primary_semantic_bbox": None,
                    "primary_semantic_compliance": "none",
                    "threat_distance": None,
                    "threat_score":    1.0,
                    "sem_source":      "none",
                    "semantic_age_ms": round(sem_age * 1000.0, 1) if sem_age != float("inf") else None,
                    "lidar_age_ms":    round(lidar_age * 1000.0, 1) if lidar_age != float("inf") else None,
                    "cmd_age_ms":      round(cmd_age * 1000.0, 1) if cmd_age != float("inf") else None,
                    "cmd_active":      cmd_active,
                    "reaction_latency_ms": None,
                    "danger_front":    False,
                    "danger_rear":     False,
                    "capped_linear":   False,
                    "capped_angular":  False,
                    "gap_steer_active": False,
                    "lv_in":           round(lv_in, 3),
                    "av_in":           round(av_in, 3),
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
                observations = self._build_pseudo_observations(frontal_pts)
                sem_source   = "lidar_fallback" if observations else "none"

            with self._reflex_lock:
                sem_cmd = self._reflex.decide(observations)
                primary_observation = self._select_primary_observation(observations, sem_source)
                primary_compliance = self._classify_observation_compliance(primary_observation)

            vulnerable_stop_override = self._should_force_vulnerable_stop(
                primary_observation,
                sem_source,
            )
            if vulnerable_stop_override and primary_observation is not None:
                profile = self._reflex.risk_profiles.get(
                    primary_observation.label,
                    self._reflex.risk_profiles["unknown"],
                )
                sem_cmd = ControlCommand(
                    action=profile.stop_action,
                    stage="REACT",
                    target_linear_x=0.0,
                    target_angular_z=0.0,
                    stop=True,
                    reason=(
                        f"vulnerable_override_{primary_observation.label}"
                        f"@{primary_observation.distance:.2f}m"
                    ),
                    dominant_label=primary_observation.label,
                    threat_distance=primary_observation.distance,
                    threat_score=max(sem_cmd.threat_score, 1.0),
                )

            if sem_source == "lidar_fallback" and sem_cmd.dominant_label == "unknown":
                sem_cmd = ControlCommand(
                    action="GO",
                    stage=sem_cmd.stage,
                    target_linear_x=NOMINAL_SPEED,
                    target_angular_z=0.0,
                    stop=False,
                    reason="lidar_fallback_deferred_to_geometry",
                    dominant_label=None,
                    threat_distance=sem_cmd.threat_distance,
                    threat_score=sem_cmd.threat_score,
                )

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
                    if vulnerable_stop_override:
                        log_parts.append(
                            f"{BOLD_RED}[VULNERABLE OVERRIDE] "
                            f"{primary_observation.label} forced stop in path at "
                            f"{primary_observation.distance:.2f} m{RESET}"
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
                    if abs(sem_cmd.target_angular_z) > 0.01:
                        steer = float(np.clip(
                            sem_cmd.target_angular_z,
                            -MAX_STEER_CONTRIB,
                            MAX_STEER_CONTRIB,
                        ))
                        av = av + steer

            out.linear.x  = lv
            out.angular.z = av

            _compliant_bypass = (
                sem_source == "yolo"
                and not sem_cmd.stop
                and sem_cmd.dominant_label in {"crop", "grass", "compliant_corridor"}
            )

            capped_linear = capped_angular = False
            geo_override_reason: Optional[str] = None
            gap_steer_active = False
            gap_bearing_deg: Optional[float] = None

            if out.linear.x > 0.01 and danger_front:
                if _compliant_bypass:
                    log_parts.append(
                        f"{BOLD_GRAY}[GEO BYPASS] {sem_cmd.dominant_label} confirmed — "
                        f"{front_count} pts, danger suppressed{RESET}"
                    )
                else:
                    out.linear.x = 0.0
                    out.angular.z = 0.0
                    geo_override_reason = f"geo_danger_front_{front_count}pts"
                    log_parts.append(
                        f"{BOLD_RED}[GEO DANGER FRONT] {front_count} pts — FULL STOP{RESET}"
                    )
            elif out.linear.x < -0.01 and danger_rear:
                out.linear.x = 0.0
                out.angular.z = 0.0
                geo_override_reason = f"geo_danger_rear_{rear_count}pts"
                log_parts.append(
                    f"{BOLD_RED}[GEO DANGER REAR] {rear_count} pts — FULL STOP{RESET}"
                )
            elif out.linear.x > 0.01 and caution_front:
                cap = TENSION_CAP_COMPLIANT if _compliant_bypass else TENSION_CAP
                out.linear.x = min(out.linear.x, cap)
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

            self._gap_bearing_history.append(self._find_gap_bearing(frontal_pts))

            output_stopped = abs(out.linear.x) < 0.01 and abs(out.angular.z) < 0.01
            if output_stopped and lv_in > 0.01:
                self._consecutive_stops += 1
            else:
                self._consecutive_stops = 0

            no_gap_steer = sem_cmd.dominant_label in {"human", "animal"}
            if (
                self._consecutive_stops >= GAP_STEER_CYCLES
                and not no_gap_steer
                and output_stopped
            ):
                valid_gaps = [b for b in self._gap_bearing_history if b is not None]
                gap_bearing: Optional[float] = None
                if len(valid_gaps) >= 2:
                    avg_gap = float(np.mean(valid_gaps))
                    if all(abs(b - avg_gap) <= 20.0 for b in valid_gaps):
                        gap_bearing = avg_gap

                if gap_bearing is not None:
                    gap_dir = 1.0 if gap_bearing > 0.0 else -1.0
                    out.angular.z = gap_dir * GAP_STEER_RATE
                    gap_steer_active = True
                    gap_bearing_deg = float(gap_bearing)
                    log_parts.append(
                        f"{BOLD_CYAN}[GAP STEER] blocked {self._consecutive_stops} cycles  "
                        f"gap={gap_bearing:.0f}° (consistent {len(valid_gaps)}/3)  "
                        f"az={out.angular.z:.2f} rad/s{RESET}"
                    )

            self._throttled_log(log_parts, out, sem_cmd, sem_source, cmd_active)

            if sem_cmd.dominant_label is None:
                compliance = "none"
            elif _compliant_bypass:
                compliance = "compliant"
            else:
                compliance = "non_compliant"

            reaction_latency_ms: Optional[float]
            if sem_source == "yolo" and sem_age != float("inf"):
                reaction_latency_ms = sem_age * 1000.0
            elif sem_source == "lidar_fallback" and lidar_age != float("inf"):
                reaction_latency_ms = lidar_age * 1000.0
            else:
                reaction_latency_ms = None

            final_action = sem_cmd.action
            final_stage = sem_cmd.stage
            final_reason = sem_cmd.reason
            decision_layer = "semantic"
            if geo_override_reason is not None:
                final_action = "STOP"
                final_stage = "GEOMETRIC"
                final_reason = geo_override_reason
                decision_layer = "geometric"
            elif gap_steer_active:
                final_action = "GAP_STEER"
                final_stage = "GEOMETRIC"
                final_reason = (
                    f"gap_bearing_{gap_bearing_deg:.1f}deg"
                    if gap_bearing_deg is not None
                    else "gap_steer"
                )
                decision_layer = "geometric"
            elif capped_linear or capped_angular:
                cap_axes = []
                if capped_linear:
                    cap_axes.append("linear")
                if capped_angular:
                    cap_axes.append("angular")
                final_action = "CAUTION_CAP"
                final_stage = "GEOMETRIC"
                final_reason = f"geo_caution_{'+'.join(cap_axes)}"
                decision_layer = "geometric"

            self.pub_cmd.publish(out)
            self.pub_decision.publish(String(data=json.dumps({
                "action":          final_action,
                "stage":           final_stage,
                "reason":          final_reason,
                "semantic_action": sem_cmd.action,
                "semantic_stage":  sem_cmd.stage,
                "semantic_reason": sem_cmd.reason,
                "semantic_status": semantic_status,
                "semantic_detection_count": semantic_detection_count,
                "decision_layer":  decision_layer,
                "dominant_label":  sem_cmd.dominant_label,
                "compliance":      compliance,
                "primary_semantic_label": primary_observation.label if primary_observation is not None else None,
                "primary_semantic_distance": round(primary_observation.distance, 3) if primary_observation is not None else None,
                "primary_semantic_confidence": round(primary_observation.confidence, 3) if primary_observation is not None else None,
                "primary_semantic_in_path": primary_observation.in_path if primary_observation is not None else None,
                "primary_semantic_source": primary_observation.source if primary_observation is not None else None,
                "primary_semantic_bbox": (
                    primary_observation.bbox.__dict__
                    if primary_observation is not None and primary_observation.bbox is not None
                    else None
                ),
                "primary_semantic_compliance": primary_compliance,
                "priority_stop_active": vulnerable_stop_override,
                "threat_distance": sem_cmd.threat_distance,
                "threat_score":    round(sem_cmd.threat_score, 3),
                "sem_source":      sem_source,
                "semantic_age_ms": round(sem_age * 1000.0, 1) if sem_age != float("inf") else None,
                "lidar_age_ms":    round(lidar_age * 1000.0, 1) if lidar_age != float("inf") else None,
                "cmd_age_ms":      round(cmd_age * 1000.0, 1) if cmd_age != float("inf") else None,
                "cmd_active":      cmd_active,
                "reaction_latency_ms": round(reaction_latency_ms, 1) if reaction_latency_ms is not None else None,
                "danger_front":    danger_front,
                "danger_rear":     danger_rear,
                "capped_linear":   capped_linear,
                "capped_angular":  capped_angular,
                "gap_steer_active": gap_steer_active,
                "gap_bearing_deg": round(gap_bearing_deg, 2) if gap_bearing_deg is not None else None,
                "lv_in":           round(lv_in, 3),
                "av_in":           round(av_in, 3),
                "lv_out":          round(out.linear.x, 3),
                "av_out":          round(out.angular.z, 3),
            })))
        finally:
            self._cycle_lock.release()

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

    # ── Polar-histogram gap finder ─────────────────────────────────────────────

    def _find_gap_bearing(
        self, pts: Optional[np.ndarray]
    ) -> Optional[float]:
        """
        VFH-style polar histogram gap finder.

        Divides the frontal ±60° arc into GAP_BIN_DEG bins.  Each bin is scored
        by the minimum LiDAR distance within it (infinity when empty — meaning
        fully open sky in that direction).  The bin with the greatest minimum
        distance is the "most open gap."

        Returns the gap bearing in degrees (positive = LEFT, same convention as
        the rest of the system) or None when:
          • No LiDAR data available.
          • The best gap is closer than GAP_MIN_CLEAR_M (not worth turning toward).
          • The best gap is within GAP_MIN_BEARING_DEG of straight ahead (already
            pointing at the gap — the obstacle is the problem, not heading).

        The caller steers TOWARD the returned bearing (sign convention is therefore
        the opposite of _steer_away_from_bearing).
        """
        if pts is None or len(pts) == 0:
            return None

        x    = pts[:, 0].astype(np.float64)
        y    = pts[:, 1].astype(np.float64)
        dist = np.hypot(x, y)

        valid = (dist >= 0.20) & (dist <= PSEUDO_MAX_RANGE_M)
        if not valid.any():
            return None
        x, y, dist = x[valid], y[valid], dist[valid]

        # positive bearing = LEFT (y > 0 in ROS robot frame)
        bearings = np.degrees(np.arctan2(y, x))

        # Restrict search to ±60° — turning more than that loses the waypoint
        fov = 60.0
        fov_mask = np.abs(bearings) <= fov
        if not fov_mask.any():
            return None
        bearings = bearings[fov_mask]
        dist     = dist[fov_mask]

        # Build polar histogram: minimum distance per angular bin
        half        = int(fov / GAP_BIN_DEG)
        n_bins      = half * 2
        bin_centers = np.linspace(
            -fov + GAP_BIN_DEG / 2.0,
             fov - GAP_BIN_DEG / 2.0,
            n_bins,
        )
        bin_min = np.full(n_bins, np.inf)

        for i, center in enumerate(bin_centers):
            lo = center - GAP_BIN_DEG / 2.0
            hi = center + GAP_BIN_DEG / 2.0
            in_bin = (bearings >= lo) & (bearings < hi)
            if in_bin.any():
                bin_min[i] = float(dist[in_bin].min())
            # else stays inf → fully open direction

        # Best gap = bin with the largest minimum distance
        best_i       = int(np.argmax(bin_min))
        gap_bearing  = float(bin_centers[best_i])
        gap_clearance = float(bin_min[best_i])

        # Reject if the gap is too shallow (not actually passable)
        if gap_clearance < GAP_MIN_CLEAR_M:
            return None

        # Reject if the gap is essentially straight ahead — the heading is already
        # correct; the problem is the obstacle depth, not the direction
        if abs(gap_bearing) < GAP_MIN_BEARING_DEG:
            return None

        return gap_bearing


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
