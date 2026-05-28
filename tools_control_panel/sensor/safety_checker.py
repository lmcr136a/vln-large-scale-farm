"""
Safety checker node — detects obstacles in 8 zones around the robot.

Robot frame convention (LiDAR frame):
  x: positive = FORWARD
  y: positive = LEFT
  z: positive = UP

Robot body: circle of radius 30 cm.

Zone layout (8 angular sectors, 45° each):
  [front-left] [front      ] [front-right]
  [left      ] [  robot    ] [right      ]
  [back-left ] [back       ] [back-right ]

Detection bands (radial distance from robot center):
  red    : 30~40 cm from center
  yellow : 40~50 cm from center
  green  : 50~60 cm from center

Z filter: -20 cm ~ +20 cm (robot body height range).

Publishes: /safety_checker (std_msgs/String, JSON) at 5 Hz.
"""

import json
import os
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

PRINT_DEBUG    = True
DEBUG_INTERVAL = 5.0
DEBUG_MAX_PTS  = 1_000
DEBUG_OUT      = './safety_debug.png'

PUBLISH_HZ = 5.0

# Geometry (metres)
ROBOT_R = 0.30   # robot body radius

Z_MIN = -0.40
Z_MAX =  0.20

# (color, r_near, r_far) — absolute distance from robot center
BANDS = [
    ('red',    0.3, 0.45),
    ('yellow', 0.45, 0.55),
    ('green',  0.55, 0.70),
]
MIN_PTS = 50

ZONES    = ['front', 'front_left', 'left', 'back_left',
            'back',  'back_right', 'right', 'front_right']
PRIORITY = {None: 0, 'green': 1, 'yellow': 2, 'red': 3}

# Zone angle ranges in robot frame (x=forward=0°, y=left=90°), CCW positive
# Each zone covers 45°
_A = np.pi / 8   # 22.5°
ZONE_ANGLES = {
    'front':       (-_A,    _A),
    'front_left':  ( _A,  3*_A),
    'left':        (3*_A, 5*_A),
    'back_left':   (5*_A, 7*_A),
    'back':        (7*_A, 9*_A),    # wraps: 157.5° to 202.5° = ±180° region
    'back_right':  (-7*_A, -5*_A),
    'right':       (-5*_A, -3*_A),
    'front_right': (-3*_A,  -_A),
}

# Visualization: screen space has x_screen=-y_robot (right), y_screen=+x_robot (up=forward)
# θ_screen = 90° - θ_robot_deg  →  zone wedge angles for matplotlib (CCW from +x_screen)
ZONE_WEDGES = {
    'front':       ( 67.5, 112.5),
    'front_left':  (112.5, 157.5),
    'left':        (157.5, 202.5),
    'back_left':   (202.5, 247.5),
    'back':        (247.5, 292.5),
    'back_right':  (292.5, 337.5),
    'right':       (337.5, 382.5),   # 382.5 = 22.5 + 360
    'front_right': ( 22.5,  67.5),
}


def _in_zone(theta: np.ndarray, a_min: float, a_max: float) -> np.ndarray:
    """True where theta (radians, -π to π) falls in [a_min, a_max], with wrap."""
    # Normalize to [-π, π]
    if a_max <= np.pi:
        return (theta >= a_min) & (theta < a_max)
    # Wrapping case (back zone crosses ±π boundary)
    a_max_w = a_max - 2 * np.pi   # e.g. 9π/8 → π/8 - 2π
    return (theta >= a_min) | (theta < a_max_w)


def detect_zones(pts: np.ndarray) -> dict:
    """pts: Nx3 float32 in robot frame (x=fwd, y=left), z already filtered."""
    result = {z: None for z in ZONES}
    x, y = pts[:, 0], pts[:, 1]

    r     = np.sqrt(x ** 2 + y ** 2)
    theta = np.arctan2(y, x)   # robot frame angle

    outside = (np.abs(x) >= ROBOT_R) | (np.abs(y) >= ROBOT_R)
    if not outside.any():
        return result
    r     = r[outside]
    theta = theta[outside]

    def update(zone, color):
        if PRIORITY[color] > PRIORITY[result[zone]]:
            result[zone] = color

    for color, r_near, r_far in BANDS:
        in_band = (r >= r_near) & (r < r_far)
        if not in_band.any():
            continue
        t_band = theta[in_band]
        for zone, (a_min, a_max) in ZONE_ANGLES.items():
            mask = _in_zone(t_band, a_min, a_max)
            if mask.sum() >= MIN_PTS:
                update(zone, color)

    return result


def _draw_debug(pts: np.ndarray, result: dict) -> None:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches

        if len(pts) > DEBUG_MAX_PTS:
            idx = np.random.choice(len(pts), DEBUG_MAX_PTS, replace=False)
            pts = pts[idx]

        # Screen space: x_s = -y_robot (right), y_s = x_robot (forward=up)
        sx = -pts[:, 1]
        sy =  pts[:, 0]
        z  =  pts[:, 2]

        CLRS = {'red': '#ff4040', 'yellow': '#ffcc00', 'green': '#33cc44', None: '#333'}

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6), facecolor='#1a1a1a')

        # ── Top-down ────────────────────────────────────────────
        ax1.set_facecolor('#111')
        ax1.scatter(sx, sy, s=1, c='white', alpha=0.35, rasterized=True)

        # Wedge zones — outermost first
        for color, r_near, r_far in reversed(BANDS):
            for zone, (t1, t2) in ZONE_WEDGES.items():
                det    = result.get(zone)
                active = PRIORITY.get(det, 0) >= PRIORITY[color]
                ax1.add_patch(mpatches.Wedge(
                    (0, 0), r_far, t1, t2,
                    width=r_far - r_near,
                    fc=CLRS[color], ec='white', lw=0.3,
                    alpha=0.65 if active else 0.12))

        # Robot body circle
        ax1.add_patch(mpatches.Rectangle(
            (-ROBOT_R, -ROBOT_R), 2 * ROBOT_R, 2 * ROBOT_R,
            fc='#777', ec='white', lw=1.2, alpha=0.9, zorder=5))
        # Triangle pointing up = +x = forward
        ax1.text(0, 0, '▲', ha='center', va='center',
                 color='white', fontsize=10, zorder=6)

        lim = BANDS[-1][2] + 0.15
        ax1.set_xlim(-lim, lim)
        ax1.set_ylim(-lim, lim)
        ax1.set_aspect('equal')
        ax1.set_title('Top-down (▲ = forward)', color='white', pad=6)
        ax1.set_xlabel('-Y → right (m)', color='white')
        ax1.set_ylabel('+X → forward (m)', color='white')
        for spine in ax1.spines.values():
            spine.set_edgecolor('#555')
        ax1.tick_params(colors='white')

        # ── Side view X-Z ───────────────────────────────────────
        ax2.set_facecolor('#111')
        ax2.scatter(pts[:, 0], z, s=1, c='cyan', alpha=0.35, rasterized=True)
        ax2.axhline(Z_MIN, color='red',    ls='--', lw=1.4,
                    label=f'Z min = {Z_MIN*100:.0f} cm')
        ax2.axhline(Z_MAX, color='orange', ls='--', lw=1.4,
                    label=f'Z max = {Z_MAX*100:.0f} cm')
        ax2.fill_between([-BANDS[-1][2] - 0.2, BANDS[-1][2] + 0.2],
                         Z_MIN, Z_MAX, color='cyan', alpha=0.05)
        ax2.set_title('Side view (X-Z)', color='white', pad=6)
        ax2.set_xlabel('X → forward (m)', color='white')
        ax2.set_ylabel('Z → up (m)', color='white')
        ax2.legend(facecolor='#333', labelcolor='white', fontsize=8)
        for spine in ax2.spines.values():
            spine.set_edgecolor('#555')
        ax2.tick_params(colors='white')

        plt.tight_layout()
        os.makedirs(os.path.dirname(DEBUG_OUT) or '.', exist_ok=True)
        plt.savefig(DEBUG_OUT, dpi=90, facecolor='#1a1a1a')
        plt.close(fig)
        print(f'[SafetyChecker] debug → {DEBUG_OUT}')
    except Exception as e:
        print(f'[SafetyChecker] debug draw error: {e}')


class SafetyChecker(Node):
    def __init__(self):
        super().__init__('safety_checker')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(
            PointCloud2, '/rslidar_points', self._pc_cb, qos)
        self.pub   = self.create_publisher(String, '/safety_checker', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)

        self._latest: dict = {z: None for z in ZONES}
        self._lock = threading.Lock()
        self._latest_pts: np.ndarray = np.empty((0, 3), dtype=np.float32)

        if PRINT_DEBUG:
            self.create_timer(DEBUG_INTERVAL, self._debug_cb)

        self.get_logger().info('SafetyChecker ready')

    def _pc_cb(self, msg: PointCloud2):
        try:
            gen = point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
            pts = np.array([(p[0], p[1], p[2]) for p in gen], dtype=np.float32)
            if pts.size == 0:
                with self._lock:
                    self._latest = {z: None for z in ZONES}
                return
            pts = pts[(pts[:, 2] > Z_MIN) & (pts[:, 2] < Z_MAX)]
            result = detect_zones(pts) if pts.size > 0 else {z: None for z in ZONES}
            with self._lock:
                self._latest     = result
                self._latest_pts = pts
        except Exception as e:
            self.get_logger().warn(f'PC error: {e}')

    def _debug_cb(self):
        with self._lock:
            pts    = self._latest_pts.copy()
            result = dict(self._latest)
        if pts.size > 0:
            threading.Thread(
                target=_draw_debug, args=(pts, result), daemon=True).start()

    def _publish(self):
        with self._lock:
            data = dict(self._latest)
        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SafetyChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()