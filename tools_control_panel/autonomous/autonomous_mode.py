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

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        pose_topic = self.config['ros2']['topics'].get('pose', '/corrected_pose')
        self.create_subscription(PoseStamped, pose_topic, self._pose_callback, qos)
        self.get_logger().info(f'AutonomousController ready — listening on {pose_topic}')

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

    def is_active(self):
        return self._active

    def start(self, waypoints):
        if self._active:
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
        if not self._active:
            return False
        self.get_logger().info('Stopping')
        self._stop_event.set()
        self._active = False
        self._zero_vel()
        return True

    def _drive_loop(self, waypoints):
        print(f'[drive_loop] started, {len(waypoints)} waypoints', flush=True)
        logger, log_path = create_session_logger()
        logger.info(f'Session started — {len(waypoints)} waypoints')

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

                for cmd in autonomous_driving.run(
                        waypoints, self.get_current_pose, params):

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

                    if dt <= 0:
                        self._zero_vel()
                        continue

                    twist           = Twist()
                    twist.linear.x  = vt
                    twist.angular.z = vr

                    deadline = time.time() + dt
                    while time.time() < deadline:
                        if self._stop_event.is_set():
                            return
                        self.pub.publish(twist)
                        time.sleep(min(0.05, max(0.0, deadline - time.time())))

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

    def _zero_vel(self):
        self.pub.publish(Twist())