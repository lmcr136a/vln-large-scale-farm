import math
import os
import time
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped

from . import autonomous_driving
from .auto_nav_logger import create_session_logger

PUBLISH_HZ      = 20.0          # cmd_vel publish rate
WATCHDOG_TIMEOUT = 0.3          # zero vel if control loop silent for this long


def _quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y ** 2 + z ** 2))


class AutonomousController(Node):
    def __init__(self, publisher, socketio, config,
                 start_recording=None, stop_recording=None):
        super().__init__('autonomous_controller')
        self.pub      = publisher
        self.socketio = socketio
        self.config   = config
        self._start_recording = start_recording
        self._stop_recording  = stop_recording

        self._pose_lock    = threading.Lock()
        self._current_pose = None
        self._active       = False
        self._stop_event   = threading.Event()
        self._thread       = None

        # shared target velocity + watchdog
        self._vel_lock      = threading.Lock()
        self._target_vt     = 0.0
        self._target_vr     = 0.0
        self._vel_updated_at = 0.0   # timestamp of last control-loop update

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        pose_topic = self.config['ros2']['topics'].get('pose', '/corrected_pose')
        self.create_subscription(PoseStamped, pose_topic, self._pose_callback, qos)
        self.get_logger().info(f'AutonomousController ready — listening on {pose_topic}')

        # publisher thread runs always; only sends non-zero when active
        self._pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._pub_thread.start()

    # ── Pose ──────────────────────────────────────────────────────────────────

    def _pose_callback(self, msg: PoseStamped):
        p, q = msg.pose.position, msg.pose.orientation
        with self._pose_lock:
            self._current_pose = {
                'x':   float(p.x),
                'y':   float(p.y),
                'z':   float(p.z),
                'yaw': _quat_to_yaw(q.x, q.y, q.z, q.w),
            }

    def get_current_pose(self):
        with self._pose_lock:
            return dict(self._current_pose) if self._current_pose else None

    # ── Velocity publisher (runs continuously at PUBLISH_HZ) ──────────────────

    def _publish_loop(self):
        interval = 1.0 / PUBLISH_HZ
        while True:
            with self._vel_lock:
                active  = self._active
                age     = time.time() - self._vel_updated_at
                tgt_vt  = self._target_vt
                tgt_vr  = self._target_vr

            if active:
                twist = Twist()
                if age < WATCHDOG_TIMEOUT:
                    twist.linear.x  = tgt_vt
                    twist.angular.z = tgt_vr
                # else zero twist — watchdog expired, stop robot
                self.pub.publish(twist)
            # inactive: publish nothing — commander handles manual cmd_vel

            time.sleep(interval)

    def _set_vel(self, vt: float, vr: float):
        with self._vel_lock:
            self._target_vt      = vt
            self._target_vr      = vr
            self._vel_updated_at = time.time()

    def _zero_vel(self):
        with self._vel_lock:
            self._target_vt      = 0.0
            self._target_vr      = 0.0
            self._vel_updated_at = 0.0   # let watchdog keep it zero

    # ── Control ───────────────────────────────────────────────────────────────

    def is_active(self):
        return self._active

    def start(self, waypoints, resume=False):
        if self._active:
            return False
        min_wp = self.config['autonomous'].get('min_waypoints', 2)
        if len(waypoints) < min_wp:
            self.get_logger().warn(f'Need >= {min_wp} waypoints (got {len(waypoints)})')
            return False
        self.get_logger().info(
            f'{"Resuming" if resume else "Starting"} — {len(waypoints)} waypoints')
        self._active = True
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._drive_loop, args=(waypoints, resume), daemon=True)
        self._thread.start()
        return True

    def _nearest_waypoint_index(self, waypoints) -> int:
        """Index of the waypoint closest to the robot's current position — used
        by Resume so we continue from where we stopped, not the first point."""
        pose = self.get_current_pose()
        if pose is None:
            return 0
        rx, ry = pose['x'], pose['y']
        return min(
            range(len(waypoints)),
            key=lambda i: (waypoints[i]['x'] - rx) ** 2 + (waypoints[i]['y'] - ry) ** 2,
        )

    def stop(self):
        if not self._active:
            return False
        self.get_logger().info('Stopping')
        self._stop_event.set()
        self._active = False
        self._zero_vel()
        return True

    def _drive_loop(self, waypoints, resume=False):
        print(f'[drive_loop] started, {len(waypoints)} waypoints', flush=True)
        logger, log_path = create_session_logger()
        logger.info(f'Session started — {len(waypoints)} waypoints (resume={resume})')

        # Resume continues from the waypoint nearest the robot; a normal Run
        # (and every lap after the first) starts from the beginning.
        first_lap_start = self._nearest_waypoint_index(waypoints) if resume else 0
        if resume:
            logger.info(f'Resuming from nearest waypoint index {first_lap_start}')

        try:
            max_laps = self.config['autonomous'].get('max_repeat_num', 1)
            params   = self.config.get('autonomous', {})

            ts      = datetime.now().strftime('%y%m%d_%H%M%S')
            rec_dir = f'auto_nav/{ts}'
            if self._start_recording:
                self._start_recording(rec_dir)
                logger.info(f'Recording started: {rec_dir}')

            self.socketio.emit('robot_status',
                               {'status': f'Navigating — {len(waypoints)} waypoints'},
                               namespace='/')

            for lap in range(max_laps):
                if self._stop_event.is_set():
                    break
                self.get_logger().info(f'Lap {lap + 1}/{max_laps}')
                logger.info(f'Lap {lap + 1}/{max_laps} started')

                lap_start = first_lap_start if lap == 0 else 0
                for cmd in autonomous_driving.run(
                        waypoints, self.get_current_pose, params,
                        start_index=lap_start):

                    if self._stop_event.is_set():
                        logger.info('Interrupted by stop signal')
                        return

                    status = cmd.get('status', '')

                    if 'waypoint_reached' in cmd:
                        idx = cmd['waypoint_reached']
                        self.get_logger().info(f'WP {idx + 1}/{len(waypoints)} reached')
                        logger.info(f'Destination point {idx + 1} reached')
                        self.socketio.emit('waypoint_reached',
                                           {'index': idx}, namespace='/')

                    if cmd.get('completed'):
                        logger.info('All waypoints completed')
                        break

                    if status:
                        self.socketio.emit('robot_status', {'status': status}, namespace='/')

                    vt = cmd.get('vt', 0.0)
                    vr = cmd.get('vr', 0.0)
                    dt = cmd.get('dt', autonomous_driving.CONTROL_DT)

                    self._set_vel(vt, vr)

                    if dt > 0:
                        deadline = time.time() + dt
                        while time.time() < deadline:
                            if self._stop_event.is_set():
                                return
                            time.sleep(0.01)

        except Exception as e:
            import traceback
            print(f'[drive_loop] EXCEPTION: {e}', flush=True)
            traceback.print_exc()
            self.get_logger().error(f'Drive loop error: {e}')
            logger.info(f'Drive loop failed: {e}')
        finally:
            self._zero_vel()
            self._active = False
            if self._stop_recording:
                try:
                    self._stop_recording()
                    logger.info('Recording stopped')
                except Exception as e:
                    logger.info(f'Recording stop error: {e}')
            self.socketio.emit('auto_mode_completed', namespace='/')
            self.socketio.emit('robot_status', {'status': ''}, namespace='/')
            self.get_logger().info('Finished')
            logger.info('Session ended')
            print('[drive_loop] finished', flush=True)