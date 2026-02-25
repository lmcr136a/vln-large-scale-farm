"""
Autonomous Mode — independent ROS2 node.

Subscribes to /path directly at LiDAR rate (~10 Hz) to maintain
a high-frequency pose cache. Completely independent of save_map.py
(which is slow due to map rendering at 1 Hz).

Architecture:
    /path topic  →  _path_callback (~10 Hz)  →  _current_pose cache
                                                       ↓
    control_server  →  start(waypoints)  →  _drive_loop (CONTROL_DT = 0.1 s)
                                                  ↓ get_current_pose()
                                            autonomous_driving.run()
                                                  ↓
                                            cmd_vel publisher
"""
import math
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import tf2_ros
from tf2_ros import TransformException
from rclpy.duration import Duration

import autonomous_driving

TARGET_FRAME = 'map'


def _quat_to_yaw(x, y, z, w):
    """Extract yaw (Z-axis rotation) from a unit quaternion."""
    return float(np.arctan2(2.0*(w*z + x*y), 1.0 - 2.0*(y**2 + z**2)))


def _compose_quaternions(xt, yt, zt, wt, xp, yp, zp, wp):
    """Quaternion product q_result = q_t * q_p."""
    return (
        wt*xp + xt*wp + yt*zp - zt*yp,
        wt*yp - xt*zp + yt*wp + zt*xp,
        wt*zp + xt*yp - yt*xp + zt*wp,
        wt*wp - xt*xp - yt*yp - zt*zp,
    )


class AutonomousController(Node):
    """
    Independent ROS2 node for pose tracking and autonomous driving.
    Does NOT depend on save_map.py at all.
    """

    def __init__(self, publisher, socketio, config):
        super().__init__('autonomous_controller')

        self.pub      = publisher
        self.socketio = socketio
        self.config   = config

        self._pose_lock    = threading.Lock()
        self._current_pose = None   # {x, y, z, yaw} in TARGET_FRAME, updated at ~10 Hz

        self._active     = False
        self._stop_event = threading.Event()
        self._thread     = None

        # TF2 to transform /path poses into TARGET_FRAME
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.path_sub = self.create_subscription(
            Path, '/path', self._path_callback, qos)

        self.get_logger().info('AutonomousController ready')

    # ── High-frequency pose update ─────────────────────────────────────────────

    def _path_callback(self, msg):
        """
        Called at LiDAR rate (~10 Hz).
        Transforms the last pose in the path into TARGET_FRAME and caches it.
        """
        if not msg.poses:
            return

        last = msg.poses[-1].pose
        pos  = last.position
        q    = last.orientation

        try:
            tf = self.tf_buffer.lookup_transform(
                TARGET_FRAME, msg.header.frame_id,
                msg.header.stamp, timeout=Duration(seconds=0.1))

            t  = tf.transform.translation
            r  = tf.transform.rotation
            rx, ry, rz, rw = r.x, r.y, r.z, r.w

            # Rotate + translate position into TARGET_FRAME
            R = np.array([
                [1-2*(ry**2+rz**2), 2*(rx*ry-rw*rz), 2*(rx*rz+rw*ry)],
                [2*(rx*ry+rw*rz),   1-2*(rx**2+rz**2), 2*(ry*rz-rw*rx)],
                [2*(rx*rz-rw*ry),   2*(ry*rz+rw*rx),   1-2*(rx**2+ry**2)],
            ])
            p_map = R @ np.array([pos.x, pos.y, pos.z]) + np.array([t.x, t.y, t.z])

            # Compose quaternions to get yaw in TARGET_FRAME
            xr, yr, zr, wr = _compose_quaternions(
                rx, ry, rz, rw, q.x, q.y, q.z, q.w)
            yaw = _quat_to_yaw(xr, yr, zr, wr)

            pose = {'x': float(p_map[0]), 'y': float(p_map[1]),
                    'z': float(p_map[2]), 'yaw': yaw}

        except TransformException:
            # TF not yet available — use raw pose as fallback
            yaw  = _quat_to_yaw(q.x, q.y, q.z, q.w)
            pose = {'x': pos.x, 'y': pos.y, 'z': pos.z, 'yaw': yaw}

        with self._pose_lock:
            self._current_pose = pose

        # Emit robot_pose to web at path callback rate (~10 Hz)
        try:
            self.socketio.emit('robot_pose', pose)
        except Exception:
            pass

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
        """
        Background thread.
        autonomous_driving.run() yields one command per CONTROL_DT (0.1 s).
        Each iteration re-reads get_current_pose() for closed-loop control.
        """
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

                    # Republish at 20 Hz within the dt window
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