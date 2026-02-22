"""
Autonomous Mode
Execute and control autonomous driving mode
"""
import os
import time
import yaml
import threading
from geometry_msgs.msg import Twist
import autonomous_driving


class AutonomousController:
    def __init__(self, publisher, socketio, config):
        self.pub     = publisher
        self.socketio = socketio
        self.config  = config

        self.auto_mode_active = False
        self.auto_thread      = None
        self.auto_stop_event  = threading.Event()

    def is_active(self):
        return self.auto_mode_active

    def get_robot_pose(self, default_x, default_y, default_yaw):
        map_dir  = os.path.expanduser(self.config['paths']['map_dir'])
        map_yaml = os.path.join(map_dir, 'map_latest.yaml')

        if os.path.exists(map_yaml):
            with open(map_yaml, 'r') as f:
                d = yaml.safe_load(f)
            return (
                d.get('robot_x',   default_x),
                d.get('robot_y',   default_y),
                d.get('robot_yaw', default_yaw),
            )
        return default_x, default_y, default_yaw

    # ★ Interruptible sleep - returns immediately when stop signal is received
    def _wait(self, timeout: float) -> bool:
        """
        Wait up to timeout seconds.
        Returns True immediately if stop_event is set (stop requested).
        Returns False after normal timeout completion.
        """
        return self.auto_stop_event.wait(timeout=timeout)

    def autonomous_control_loop(self, robot_x, robot_y, robot_yaw, waypoints):
        max_repeat = self.config['autonomous']['max_repeat_num']

        self.socketio.emit('robot_status', {
            'status': f'Navigating to {len(waypoints)} waypoints'
        }, namespace='/')

        print(f"Starting autonomous driving from ({robot_x:.2f}, {robot_y:.2f})")

        repeat_count = 0
        try:
            while repeat_count < max_repeat:
                for control_cmd in autonomous_driving.run(
                    waypoints,
                    lambda: self.get_robot_pose(robot_x, robot_y, robot_yaw),
                    params=self.config.get('autonomous', {}),
                ):
                    # ── Check for stop signal ──────────────────────────────
                    if self.auto_stop_event.is_set():
                        print("Autonomous driving stopped by user")
                        self.socketio.emit('robot_status',
                                           {'status': 'Stopped by user'},
                                           namespace='/')
                        return

                    vt     = control_cmd.get('vt', 0.0)
                    vr     = control_cmd.get('vr', 0.0)
                    tt     = control_cmd.get('tt', 0.0)
                    tr     = control_cmd.get('tr', 0.0)
                    wp_idx = control_cmd.get('waypoint_reached', -1)

                    if wp_idx >= 0:
                        print(f"Reached waypoint {wp_idx + 1}/{len(waypoints)}")
                        self.socketio.emit('waypoint_reached',
                                           {'index': wp_idx},
                                           namespace='/')

                    # Continuously republish Twist for the duration (tt + tr)
                    # Scout requires ongoing cmd_vel to keep moving
                    PUBLISH_INTERVAL = 0.05   # 20 Hz republish
                    total_wait = tt + tr
                    deadline   = time.time() + total_wait

                    twist          = Twist()
                    twist.linear.x  = vt
                    twist.angular.z = vr

                    if total_wait <= 0:
                        # waypoint_reached stop command - just publish zero once
                        self.pub.publish(twist)
                    else:
                        while time.time() < deadline:
                            if self.auto_stop_event.is_set():
                                print("Autonomous driving interrupted during motion")
                                return
                            self.pub.publish(twist)
                            remaining = deadline - time.time()
                            time.sleep(min(PUBLISH_INTERVAL, max(0, remaining)))

                if self.auto_stop_event.is_set():
                    break

                repeat_count += 1
                if repeat_count < max_repeat:
                    print(f"Starting lap {repeat_count + 1}/{max_repeat}")

        except Exception as e:
            print(f"Autonomous driving error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            twist = Twist()
            self.pub.publish(twist)
            self.auto_mode_active = False
            self.socketio.emit('auto_mode_completed', namespace='/')
            self.socketio.emit('robot_status', {'status': ''}, namespace='/')
            print("Autonomous driving completed")

    def start(self, robot_x, robot_y, robot_yaw, waypoints):
        if self.auto_mode_active:
            print("Autonomous mode already active")
            return False

        if len(waypoints) < self.config['autonomous']['min_waypoints']:
            print(f"Need at least {self.config['autonomous']['min_waypoints']} waypoints")
            return False

        print(f"Starting autonomous mode with {len(waypoints)} waypoints")
        self.auto_mode_active = True
        self.auto_stop_event.clear()

        self.auto_thread = threading.Thread(
            target=self.autonomous_control_loop,
            args=(robot_x, robot_y, robot_yaw, waypoints),
            daemon=True
        )
        self.auto_thread.start()
        return True

    def stop(self):
        if not self.auto_mode_active:
            return False

        print("Stopping autonomous mode")
        self.auto_stop_event.set()
        self.auto_mode_active = False

        twist = Twist()
        self.pub.publish(twist)
        return True