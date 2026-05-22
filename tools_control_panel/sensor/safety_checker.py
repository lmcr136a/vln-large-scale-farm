"""
Safety checker node — detects obstacles in 8 zones around the robot.

Robot frame convention (user-defined):
  x: positive = RIGHT
  y: positive = FORWARD
  z: positive = UP

Robot body: ±30 cm in x and y (60×60 cm).

Zone layout:
  [front-left] [front      ] [front-right]
  [left      ] [  robot    ] [right      ]
  [back-left ] [back       ] [back-right ]

Detection bands (from robot edge, same for sides and corners):
  red    : 30~40 cm  →  60~70 cm from center
  yellow : 40~50 cm  →  70~80 cm from center
  green  : 50~60 cm  →  80~90 cm from center

Z filter: -40 cm ~ +20 cm (robot body height range).

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

PRINT_DEBUG    = False
DEBUG_INTERVAL = 5.0              # seconds between debug renders
DEBUG_MAX_PTS  = 10_000           # max points sampled for visualization
DEBUG_OUT      = './safety_debug.png'

PUBLISH_HZ  = 5.0

# ── LiDAR extrinsic (robot frame mapping) ────────────────────
# Edit to match your LiDAR mounting orientation.
# Input:  px, py, pz  = raw LiDAR point coordinates
# Output: x,  y,  z   = robot frame coordinates
#
#   x = y   # lidar's x is actual y of the robot
#   y = -x
#   z = z

def _to_robot_frame(pts: np.ndarray) -> np.ndarray:
    out = np.empty_like(pts)
    out[:, 0] =  pts[:, 1]   # x = y
    out[:, 1] = -pts[:, 0]   # y = -x
    out[:, 2] =  pts[:, 2]   # z = z
    return out

# Geometry (metres)
ROBOT_HALF  = 0.30   # half side of 60×60 cm robot
SIDE_SPAN   = 0.20   # ±20 cm y-range covered by side zones
Z_MIN       = -0.2  # m — points below this are ignored
Z_MAX       =  0.20  # m — points above this are ignored

# (color, near, far) — distance FROM robot edge, metres
BANDS = [
    ('red',    0.00, 0.10),   # right at edge → 0.30~0.40m from center
    ('yellow', 0.10, 0.20),   # 0.40~0.50m from center
    ('green',  0.20, 0.30),   # 0.50~0.60m from center
]
MIN_PTS = 5   # minimum points in a zone to trigger detection (noise filter)

ZONES    = ['front', 'front_right', 'right', 'back_right',
            'back',  'back_left',   'left',  'front_left']
PRIORITY = {None: 0, 'green': 1, 'yellow': 2, 'red': 3}


def detect_zones(pts: np.ndarray) -> dict:
    """pts: Nx3 float32 in robot frame (m), z already filtered."""
    result = {z: None for z in ZONES}
    x, y   = pts[:, 0], pts[:, 1]

    # Ignore points inside the robot body (60×60 cm footprint)
    outside = np.maximum(np.abs(x), np.abs(y)) >= ROBOT_HALF
    if not outside.any():
        return result
    pts = pts[outside]
    x, y = pts[:, 0], pts[:, 1]

    def update(zone, color):
        if PRIORITY[color] > PRIORITY[result[zone]]:
            result[zone] = color

    for color, dn, df in BANDS:
        near = ROBOT_HALF + dn
        far  = ROBOT_HALF + df

        # Right
        m = (x >= near) & (x < far) & (np.abs(y) <= SIDE_SPAN)
        if m.sum() >= MIN_PTS: update('right', color)

        # Left
        m = (x <= -near) & (x > -far) & (np.abs(y) <= SIDE_SPAN)
        if m.sum() >= MIN_PTS: update('left', color)

        # Front
        m = (y >= near) & (y < far) & (np.abs(x) <= SIDE_SPAN)
        if m.sum() >= MIN_PTS: update('front', color)

        # Back
        m = (y <= -near) & (y > -far) & (np.abs(x) <= SIDE_SPAN)
        if m.sum() >= MIN_PTS: update('back', color)

        # Corners — Chebyshev distance from robot corner
        for zone, sx, sy in [
            ('front_right',  1,  1),
            ('front_left',  -1,  1),
            ('back_right',   1, -1),
            ('back_left',   -1, -1),
        ]:
            in_quad = (sx * x > SIDE_SPAN) & (sy * y > SIDE_SPAN)
            if not in_quad.any():
                continue
            d = np.maximum(sx * x[in_quad] - ROBOT_HALF,
                           sy * y[in_quad] - ROBOT_HALF)
            if ((d >= dn) & (d < df)).sum() >= MIN_PTS:
                update(zone, color)

    return result


def _draw_debug(pts: np.ndarray, result: dict) -> None:
    """Render top-down and Z-X debug views and save to DEBUG_OUT."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches

        if len(pts) > DEBUG_MAX_PTS:
            idx = np.random.choice(len(pts), DEBUG_MAX_PTS, replace=False)
            pts = pts[idx]

        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

        CLRS  = {'red': '#ff4040', 'yellow': '#ffcc00', 'green': '#33cc44', None: '#333'}
        ALPHA = {'red': 0.55, 'yellow': 0.55, 'green': 0.55, None: 0.12}

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6), facecolor='#1a1a1a')

        # ── Top-down X-Y ────────────────────────────────────────
        ax1.set_facecolor('#111')
        ax1.scatter(x, y, s=1, c='white', alpha=0.35, rasterized=True)

        # Draw zones outermost-first so closer bands overlay farther ones
        for color, dn, df in reversed(BANDS):
            n, f = ROBOT_HALF + dn, ROBOT_HALF + df
            w = f - n   # band thickness

            # 4 sides
            for zone, rx, ry, rw, rh in [
                ('right', n,  -SIDE_SPAN, w, 2 * SIDE_SPAN),
                ('left',  -f, -SIDE_SPAN, w, 2 * SIDE_SPAN),
                ('front', -SIDE_SPAN,  n, 2 * SIDE_SPAN, w),
                ('back',  -SIDE_SPAN, -f, 2 * SIDE_SPAN, w),
            ]:
                det    = result.get(zone)
                active = PRIORITY.get(det, 0) >= PRIORITY[color]
                ax1.add_patch(mpatches.Rectangle(
                    (rx, ry), rw, rh,
                    fc=CLRS[color],
                    ec='white', lw=0.3,
                    alpha=0.65 if active else 0.12))

            # 4 corners: L-shape = two rectangles, always positive dims
            for zone, sx, sy in [
                ('front_right',  1,  1), ('front_left', -1,  1),
                ('back_right',   1, -1), ('back_left',  -1, -1),
            ]:
                det    = result.get(zone)
                active = PRIORITY.get(det, 0) >= PRIORITY[color]
                kw = dict(fc=CLRS[color], ec='white', lw=0.3,
                          alpha=0.65 if active else 0.12)

                # anchor = lower-left corner (min x, min y), dims always positive
                # Part 1: strip along sx axis
                x1 = n  if sx > 0 else -f        # left edge
                y1 = SIDE_SPAN if sy > 0 else -f  # bottom edge
                ax1.add_patch(mpatches.Rectangle(
                    (x1, y1), w, f - SIDE_SPAN, **kw))

                # Part 2: strip along sy axis (no overlap with Part 1)
                x2 = SIDE_SPAN if sx > 0 else -n  # left edge
                y2 = n if sy > 0 else -f           # bottom edge
                ax1.add_patch(mpatches.Rectangle(
                    (x2, y2), n - SIDE_SPAN, w, **kw))

        # Robot body
        ax1.add_patch(mpatches.Rectangle(
            (-ROBOT_HALF, -ROBOT_HALF), 2 * ROBOT_HALF, 2 * ROBOT_HALF,
            fc='#777', ec='white', lw=1.2, alpha=0.9, zorder=5))
        ax1.text(0, 0, '▲', ha='center', va='center',
                 color='white', fontsize=10, zorder=6)

        lim = ROBOT_HALF + BANDS[-1][2] + 0.15
        ax1.set_xlim(-lim, lim)
        ax1.set_ylim(-lim, lim)
        ax1.set_aspect('equal')
        ax1.set_title('Top-down (X-Y)', color='white', pad=6)
        ax1.set_xlabel('X → right (m)', color='white')
        ax1.set_ylabel('Y → forward (m)', color='white')
        for spine in ax1.spines.values():
            spine.set_edgecolor('#555')
        ax1.tick_params(colors='white')

        # ── Z-X plane ────────────────────────────────────────────
        ax2.set_facecolor('#111')
        ax2.scatter(x, z, s=1, c='cyan', alpha=0.35, rasterized=True)
        ax2.axhline(Z_MIN, color='red',    ls='--', lw=1.4,
                    label=f'Z min = {Z_MIN*100:.0f} cm')
        ax2.axhline(Z_MAX, color='orange', ls='--', lw=1.4,
                    label=f'Z max = {Z_MAX*100:.0f} cm')
        ax2.fill_between(ax2.get_xlim() or [-2, 2],
                         Z_MIN, Z_MAX, color='cyan', alpha=0.05)
        ax2.set_title('Z-X plane', color='white', pad=6)
        ax2.set_xlabel('X → right (m)', color='white')
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
            pts = _to_robot_frame(pts)
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