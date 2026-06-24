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

# Start-of-run heading nudge: drive straight forward so the GPS localizer can
# confirm the absolute heading from travel (it locks after ~1 m). Done only on a
# fresh RUN when the heading isn't established yet — never on Resume.
HEADING_NUDGE_DIST  = 1.0       # m — stop the nudge once this far
HEADING_NUDGE_TIME  = 2.0       # s — or after this long, whichever comes first
HEADING_NUDGE_SPEED = 0.5       # m/s — ~1 m in ~2 s


def _quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y ** 2 + z ** 2))


class AutonomousController(Node):
    def __init__(self, publisher, socketio, config,
                 start_recording=None, stop_recording=None, heading_check=None):
        super().__init__('autonomous_controller')
        self.pub      = publisher
        self.socketio = socketio
        self.config   = config
        self._start_recording = start_recording
        self._stop_recording  = stop_recording
        # Returns True once the GPS heading is locked. Used to skip the
        # start-of-run forward nudge when the direction is already established.
        self._heading_check   = heading_check

        self._pose_lock    = threading.Lock()
        self._current_pose = None
        self._active       = False
        self._stop_event   = threading.Event()
        # Paused (set) = mission stays active but stops issuing/publishing velocity,
        # so SafetyGuard can take over (back away from a red zone) without ending
        # the run. Cleared again → path following continues from where it held.
        self._pause_event  = threading.Event()
        self._thread       = None

        # shared target velocity + watchdog
        self._vel_lock      = threading.Lock()
        self._target_vt     = 0.0
        self._target_vr     = 0.0
        self._vel_updated_at = 0.0   # timestamp of last control-loop update
        # Last velocity actually commanded by the drive loop — read by SafetyGuard
        # to reverse the motion (e.g. was turning left → turn right back).
        self._last_cmd_vt   = 0.0
        self._last_cmd_vr   = 0.0

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

            # While paused, publish nothing so the Commander's safety override
            # (reverse / back-away) is the sole writer of cmd_vel.
            if active and not self._pause_event.is_set():
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
            if vt != 0.0 or vr != 0.0:
                self._last_cmd_vt = vt
                self._last_cmd_vr = vr

    def _zero_vel(self):
        with self._vel_lock:
            self._target_vt      = 0.0
            self._target_vr      = 0.0
            self._vel_updated_at = 0.0   # let watchdog keep it zero

    def last_command(self):
        """Most recent non-zero (vt, vr) the drive loop issued. SafetyGuard uses
        this to reverse the motion when a close object appears."""
        with self._vel_lock:
            return self._last_cmd_vt, self._last_cmd_vr

    # ── Pause / resume (SafetyGuard back-away without ending the mission) ──────

    def pause(self):
        """Suspend velocity output but keep the mission active. Idempotent."""
        if self._active and not self._pause_event.is_set():
            self._pause_event.set()
            self._zero_vel()
            self.get_logger().info('Autonomous paused (safety) — mission still active')

    def resume(self):
        """Resume path following after a pause. Idempotent."""
        if self._pause_event.is_set():
            self._pause_event.clear()
            self.get_logger().info('Autonomous resumed after safety hold')

    def is_paused(self):
        return self._pause_event.is_set()

    # ── Control ───────────────────────────────────────────────────────────────

    def is_active(self):
        return self._active

    def start(self, waypoints, resume=False, start_index=None):
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
        self._pause_event.clear()
        self._thread = threading.Thread(
            target=self._drive_loop, args=(waypoints, resume, start_index), daemon=True)
        self._thread.start()
        return True

    def _nearest_waypoint_index(self, waypoints) -> int:
        """Index of the closest *upcoming* waypoint — used by Resume so we head to
        the next point ahead, never one the robot has already driven past.

        We take the nearest waypoint, then if the robot has already passed it
        (its position projects beyond that waypoint, toward the following one),
        advance to the next index instead of re-targeting a point behind us."""
        pose = self.get_current_pose()
        # After a restart the first pose may not have arrived yet; wait briefly so
        # Resume heads to the nearest upcoming point instead of falling back to wp 0.
        deadline = time.time() + 3.0
        while pose is None and time.time() < deadline and not self._stop_event.is_set():
            time.sleep(0.05)
            pose = self.get_current_pose()
        if pose is None:
            return 0
        rx, ry = pose['x'], pose['y']
        n = len(waypoints)
        i = min(
            range(n),
            key=lambda j: (waypoints[j]['x'] - rx) ** 2 + (waypoints[j]['y'] - ry) ** 2,
        )
        if i < n - 1:
            ax, ay = waypoints[i]['x'],     waypoints[i]['y']
            bx, by = waypoints[i + 1]['x'], waypoints[i + 1]['y']
            seg_x, seg_y = bx - ax, by - ay
            # Robot projects past wp[i] along the leg toward wp[i+1] → already passed it.
            if (rx - ax) * seg_x + (ry - ay) * seg_y > 0:
                return i + 1
        return i

    def stop(self):
        if not self._active:
            return False
        self.get_logger().info('Stopping')
        self._stop_event.set()
        self._pause_event.clear()
        self._active = False
        self._zero_vel()
        return True

    def _establish_heading_nudge(self, logger):
        """Drive straight forward (≤ HEADING_NUDGE_DIST or HEADING_NUDGE_TIME) so
        the GPS localizer can confirm the absolute heading from travel. Stops early
        if the heading locks first. Honours stop/pause like the main loop."""
        start = self.get_current_pose()
        deadline = time.time() + 3.0
        while start is None and time.time() < deadline and not self._stop_event.is_set():
            time.sleep(0.05)
            start = self.get_current_pose()
        sx, sy = (start['x'], start['y']) if start else (None, None)

        self.get_logger().info('Heading nudge — driving forward to lock GPS heading')
        logger.info('Heading nudge started (forward to establish GPS heading)')
        self.socketio.emit('robot_status',
                           {'status': 'Establishing heading — driving forward'},
                           namespace='/')

        t0 = time.time()
        while not self._stop_event.is_set():
            # Hold here if SafetyGuard paused us (back-away) — don't count the time.
            while self._pause_event.is_set() and not self._stop_event.is_set():
                self._zero_vel()
                time.sleep(0.05)
                t0 = time.time()   # restart the clock once we resume
            if self._stop_event.is_set():
                break
            if self._heading_check and self._heading_check():
                logger.info('Heading nudge: heading locked early')
                break
            if time.time() - t0 >= HEADING_NUDGE_TIME:
                break
            pose = self.get_current_pose()
            if pose and sx is not None:
                if math.hypot(pose['x'] - sx, pose['y'] - sy) >= HEADING_NUDGE_DIST:
                    break
            self._set_vel(HEADING_NUDGE_SPEED, 0.0)
            time.sleep(autonomous_driving.CONTROL_DT)

        self._zero_vel()
        logger.info('Heading nudge complete')

    def _drive_loop(self, waypoints, resume=False, start_index=None):
        print(f'[drive_loop] started, {len(waypoints)} waypoints', flush=True)
        logger, log_path = create_session_logger()
        logger.info(f'Session started — {len(waypoints)} waypoints (resume={resume})')

        # First-lap start point:
        #  • explicit start_index (from the "from pt" input) wins, clamped to range
        #  • else Resume → nearest upcoming waypoint
        #  • else (normal Run) → from the beginning
        if start_index is not None:
            first_lap_start = max(0, min(int(start_index), len(waypoints) - 1))
            logger.info(f'Starting from requested waypoint index {first_lap_start}')
        elif resume:
            first_lap_start = self._nearest_waypoint_index(waypoints)
            logger.info(f'Resuming from nearest waypoint index {first_lap_start}')
        else:
            first_lap_start = 0

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

            # Heading nudge: only on a fresh RUN and only if not already locked.
            # (Resume / "from pt" keep the heading already established earlier.)
            if not resume:
                established = self._heading_check() if self._heading_check else False
                if not established:
                    self._establish_heading_nudge(logger)

            for lap in range(max_laps):
                if self._stop_event.is_set():
                    break
                self.get_logger().info(f'Lap {lap + 1}/{max_laps}')
                logger.info(f'Lap {lap + 1}/{max_laps} started')

                lap_start = first_lap_start if lap == 0 else 0
                gen = autonomous_driving.run(
                    waypoints, self.get_current_pose, params,
                    start_index=lap_start)
                while True:
                    # Hold here while paused (SafetyGuard backing away). The
                    # generator is NOT advanced, so when we resume it recomputes
                    # from the robot's current pose toward the same next waypoint.
                    while self._pause_event.is_set() and not self._stop_event.is_set():
                        time.sleep(0.05)
                    if self._stop_event.is_set():
                        logger.info('Interrupted by stop signal')
                        return
                    try:
                        cmd = next(gen)
                    except StopIteration:
                        break

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