"""
Autonomous Mode — independent ROS2 node.

Subscribes to /localized_pose (same topic as save_map_glim.py) to maintain
a high-frequency pose cache AND emit robot_pose directly via socketio.

Architecture:
    /localized_pose  →  _pose_callback  →  _current_pose cache
                                        →  socketio.emit('robot_pose')  [real-time]

    control_server  →  start(waypoints)  →  _drive_loop (CONTROL_DT = 0.1 s)
                                                   ↓ get_current_pose()
                                             autonomous_driving.run()
                                                   ↓
                                             cmd_vel publisher
"""
import os
import time
import threading
import json

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped

from . import autonomous_driving


def _quat_to_yaw(x, y, z, w):
    """Extract yaw (Z-axis rotation) from a unit quaternion."""
    return float(np.arctan2(2.0*(w*z + x*y), 1.0 - 2.0*(y**2 + z**2)))


class AutonomousController(Node):
    """
    Independent ROS2 node for pose tracking and autonomous driving.
    Subscribes to /localized_pose (published by localizer_ext in both
    localization and new-mapping modes).
    Emits robot_pose via socketio directly — no file I/O delay.
    """

    def __init__(self, publisher, socketio, config):
        super().__init__('autonomous_controller')

        self.pub      = publisher
        self.socketio = socketio
        self.config   = config

        self._pose_lock    = threading.Lock()
        self._current_pose = None   # {x, y, z, yaw} in map frame

        self._active     = False
        self._stop_event = threading.Event()
        self._thread     = None

        # Map rotation angle (PCA from save_map_glim.py) — cached from map_state.json
        self._rot_angle       = 0.0
        self._rot_angle_mtime = 0.0
        map_dir = os.path.expanduser(self.config['paths']['map_dir'])
        map_state_file = self.config['paths'].get('map_state', 'map_state.json')
        self._map_yaml_path   = os.path.join(map_dir, map_state_file)

        pose_topic = self.config['ros2']['topics']['pose']
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self._pose_callback, qos)

        self.get_logger().info(f'AutonomousController ready — listening on {pose_topic}')

    # ── Pose update + real-time socketio emit ──────────────────────────────────

    def _pose_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        q   = msg.pose.orientation
        yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)

        # Refresh rot_angle from map_state.json only when the file changes
        try:
            mtime = os.path.getmtime(self._map_yaml_path)
            if mtime > self._rot_angle_mtime:
                with open(self._map_yaml_path, 'r') as f:
                    meta = json.load(f)
                self._rot_angle       = float(meta.get('rot_angle', 0.0))
                self._rot_angle_mtime = mtime
        except Exception:
            pass

        # Apply map rotation — SAME transform as frontend worldToStagePixel
        theta = self._rot_angle
        c, s  = np.cos(theta), np.sin(theta)
        rx    =  c * float(pos.x) + s * float(pos.y)
        ry    = -s * float(pos.x) + c * float(pos.y)
        ryaw  = yaw - theta

        # _current_pose in rotated frame — waypoints from JS are also in rotated frame
        # so autonomous_driving.py compares apples-to-apples
        with self._pose_lock:
            self._current_pose = {
                'x':   rx,
                'y':   ry,
                'z':   float(pos.z),
                'yaw': ryaw,
            }

        # Emit same rotated coords to frontend
        self.socketio.emit('robot_pose', {
            'x':   rx,
            'y':   ry,
            'z':   float(pos.z),
            'yaw': ryaw,
        }, namespace='/')

    def get_current_pose(self):
        """Thread-safe. Returns {x, y, z, yaw} in map frame, or None."""
        with self._pose_lock:
            return dict(self._current_pose) if self._current_pose is not None else None

    # ── Public API ─────────────────────────────────────────────────────────────

    def is_active(self):
        return self._active

    def start(self, waypoints):
        """Start autonomous driving. waypoints: list of {x, y} dicts in map frame."""
        if self._active:
            self.get_logger().warn('Already active')
            return False

        min_wp = self.config['autonomous']['min_waypoints']
        if len(waypoints) < min_wp:
            self.get_logger().warn(f'Need >= {min_wp} waypoints (got {len(waypoints)})')
            return False

        self.get_logger().info(f'Starting — {len(waypoints)} waypoints')
        self._active = True
        self._stop_event.clear()

        self._thread = threading.Thread(
            target=self._drive_loop, args=(waypoints,), daemon=True)
        self._thread.start()
        return True

    def stop(self):
        """Interrupt the drive loop and publish zero velocity."""
        if not self._active:
            return False
        self.get_logger().info('Stopping')
        self._stop_event.set()
        self._active = False
        self._zero_vel()
        return True

    # ── Drive loop ─────────────────────────────────────────────────────────────

    def _drive_loop(self, waypoints):
        max_laps = self.config['autonomous'].get('max_repeat_num', 1)
        params   = self.config.get('autonomous', {})

        self.socketio.emit('robot_status',
                           {'status': f'Navigating — {len(waypoints)} waypoints'},
                           namespace='/')
        try:
            for lap in range(max_laps):
                if self._stop_event.is_set():
                    break

                self.get_logger().info(f'Lap {lap+1}/{max_laps}')

                for cmd in autonomous_driving.run(
                        waypoints, self.get_current_pose, params):

                    if self._stop_event.is_set():
                        self.get_logger().info('Interrupted')
                        return

                    if 'waypoint_reached' in cmd:
                        idx = cmd['waypoint_reached']
                        self.get_logger().info(
                            f'WP {idx+1}/{len(waypoints)} reached')
                        self.socketio.emit('waypoint_reached',
                                           {'index': idx}, namespace='/')

                    if cmd.get('completed', False):
                        break

                    vt = cmd.get('vt', 0.0)
                    vr = cmd.get('vr', 0.0)
                    dt = cmd.get('dt', autonomous_driving.CONTROL_DT)

                    if dt <= 0:
                        self._zero_vel()
                        continue

                    twist           = Twist()
                    twist.linear.x  = vt
                    twist.angular.z = vr

                    deadline   = time.time() + dt
                    PUB_PERIOD = 0.05
                    while time.time() < deadline:
                        if self._stop_event.is_set():
                            return
                        self.pub.publish(twist)
                        time.sleep(min(PUB_PERIOD, max(0.0, deadline - time.time())))

        except Exception as e:
            self.get_logger().error(f'Drive loop error: {e}')
            import traceback
            traceback.print_exc()
        finally:
            self._zero_vel()
            self._active = False
            self.socketio.emit('auto_mode_completed', namespace='/')
            self.socketio.emit('robot_status', {'status': ''}, namespace='/')
            self.get_logger().info('Finished')

    def _zero_vel(self):
        self.pub.publish(Twist())