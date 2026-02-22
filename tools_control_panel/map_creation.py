"""
Map Creation Mode
Map creation mode: keyboard input handling and robot control
"""
import os
import time
import rclpy
import threading
from geometry_msgs.msg import Twist


# os.system() is only called when PTZ value changes by more than this threshold
PTZ_THRESHOLD = 1.0


class MapCreationController:
    def __init__(self, publisher, config):
        self.pub    = publisher
        self.config = config
        self.down_keys          = set()
        self.should_update_twist = True

        self.linear_speed  = config['robot']['linear_speed']
        self.angular_speed = config['robot']['angular_speed']

        # Current PTZ target values
        self.pan  = 0
        self.tilt = 0
        self.zoom = 0
        # ★ Last values actually sent via os.system()
        self._sent_pan  = None
        self._sent_tilt = None
        self._sent_zoom = None
        self.ptz_lock = threading.Lock()

    def get_linear_speed(self):
        return self.linear_speed

    def handle_keydown(self, key, socketio):
        print(f"Key down: {key}")
        self.down_keys.add(key)
        self.should_update_twist = True

        updated = False
        if key == ',':
            self.linear_speed = max(
                self.config['robot']['min_linear_speed'],
                self.linear_speed - 0.1
            )
            updated = True
        elif key == '.':
            self.linear_speed = min(
                self.config['robot']['max_linear_speed'],
                self.linear_speed + 0.1
            )
            updated = True
        elif key == '[':
            self.angular_speed = max(
                self.config['robot']['min_angular_speed'],
                self.angular_speed - 0.1
            )
            updated = True
        elif key == ']':
            self.angular_speed = min(
                self.config['robot']['max_angular_speed'],
                self.angular_speed + 0.1
            )
            updated = True

        if updated:
            socketio.emit('speed_update', {
                'linear':  self.linear_speed,
                'angular': self.angular_speed,
            })

    def handle_keyup(self, key):
        print(f"Key up: {key}")
        self.down_keys.discard(key)
        twist = Twist()
        self.pub.publish(twist)

    def clear_keys_and_stop(self):
        self.down_keys.clear()
        self.should_update_twist = False
        twist = Twist()
        try:
            if rclpy.ok():
                self.pub.publish(twist)
        except Exception as e:
            print(f"Failed to publish stop command: {e}")

    def update_ptz(self):
        """Update PTZ - only calls os.system() when change exceeds threshold"""
        with self.ptz_lock:
            if 'w' in self.down_keys:
                self.tilt = min(self.tilt + self.config['ptz']['tilt_step'],
                                self.config['ptz']['tilt_max'])
            if 's' in self.down_keys:
                self.tilt = max(self.tilt - self.config['ptz']['tilt_step'],
                                self.config['ptz']['tilt_min'])
            if 'a' in self.down_keys:
                self.pan = min(self.pan + self.config['ptz']['pan_step'],
                               self.config['ptz']['pan_max'])
            if 'd' in self.down_keys:
                self.pan = max(self.pan - self.config['ptz']['pan_step'],
                               self.config['ptz']['pan_min'])
            if 'x' in self.down_keys:
                self.zoom = min(self.zoom + self.config['ptz']['zoom_step'],
                                self.config['ptz']['zoom_max'])
            if 'z' in self.down_keys:
                self.zoom = max(self.zoom - self.config['ptz']['zoom_step'],
                                self.config['ptz']['zoom_min'])

            # ★ Compare against last sent values - only send if change exceeds threshold
            pan_changed  = self._sent_pan  is None or abs(self.pan  - self._sent_pan)  > PTZ_THRESHOLD
            tilt_changed = self._sent_tilt is None or abs(self.tilt - self._sent_tilt) > PTZ_THRESHOLD
            zoom_changed = self._sent_zoom is None or abs(self.zoom - self._sent_zoom) > PTZ_THRESHOLD

            if pan_changed or tilt_changed or zoom_changed:
                cmd = (f"v4l2-ctl --set-ctrl=pan_absolute={self.pan} "
                       f"--set-ctrl=tilt_absolute={self.tilt} "
                       f"--set-ctrl=zoom_absolute={self.zoom}")
                os.system(cmd)
                self._sent_pan  = self.pan
                self._sent_tilt = self.tilt
                self._sent_zoom = self.zoom

    def update_loop(self, auto_mode_checker):
        while True:
            if auto_mode_checker():
                time.sleep(0.1)
                continue

            if not self.should_update_twist:
                time.sleep(0.1)
                continue

            self.update_ptz()

            twist = Twist()
            if 'ArrowUp'    in self.down_keys:
                twist.linear.x  += self.linear_speed
            if 'ArrowDown'  in self.down_keys:
                twist.linear.x  -= self.linear_speed
            if 'ArrowLeft'  in self.down_keys:
                twist.angular.z += self.angular_speed
            if 'ArrowRight' in self.down_keys:
                twist.angular.z -= self.angular_speed

            try:
                if rclpy.ok():
                    self.pub.publish(twist)
                else:
                    exit()
            except Exception as e:
                print(f"Failed to publish in update_loop: {e}")
                exit()

            time.sleep(0.1)