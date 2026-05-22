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
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

PUBLISH_HZ  = 5.0

# Geometry (metres)
ROBOT_HALF  = 0.30   # half side of 60×60 cm robot
SIDE_SPAN   = 0.45   # ±45 cm y-range covered by side zones
Z_MIN       = -0.40  # m
Z_MAX       =  0.20  # m

# (color, near, far) — distance FROM robot edge, metres
BANDS = [
    ('red',    0.30, 0.40),
    ('yellow', 0.40, 0.50),
    ('green',  0.50, 0.60),
]

ZONES    = ['front', 'front_right', 'right', 'back_right',
            'back',  'back_left',   'left',  'front_left']
PRIORITY = {None: 0, 'green': 1, 'yellow': 2, 'red': 3}


def detect_zones(pts: np.ndarray) -> dict:
    """pts: Nx3 float32 in robot frame (m), z already filtered."""
    result = {z: None for z in ZONES}
    x, y   = pts[:, 0], pts[:, 1]

    def update(zone, color):
        if PRIORITY[color] > PRIORITY[result[zone]]:
            result[zone] = color

    for color, dn, df in BANDS:
        near = ROBOT_HALF + dn
        far  = ROBOT_HALF + df

        # Right
        m = (x >= near) & (x < far) & (np.abs(y) <= SIDE_SPAN)
        if m.any(): update('right', color)

        # Left
        m = (x <= -near) & (x > -far) & (np.abs(y) <= SIDE_SPAN)
        if m.any(): update('left', color)

        # Front
        m = (y >= near) & (y < far) & (np.abs(x) <= SIDE_SPAN)
        if m.any(): update('front', color)

        # Back
        m = (y <= -near) & (y > -far) & (np.abs(x) <= SIDE_SPAN)
        if m.any(): update('back', color)

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
            if ((d >= dn) & (d < df)).any():
                update(zone, color)

    return result


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
            pts = pts[(pts[:, 2] >= Z_MIN) & (pts[:, 2] <= Z_MAX)]
            result = detect_zones(pts) if pts.size > 0 else {z: None for z in ZONES}
            with self._lock:
                self._latest = result
        except Exception as e:
            self.get_logger().warn(f'PC error: {e}')

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