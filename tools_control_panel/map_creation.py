"""
Map Creation Mode
Map creation mode: keyboard input handling and robot control
"""
import os
import time
import threading
from geometry_msgs.msg import Twist


class MapCreationController:
    def __init__(self, publisher, config):
        self.pub = publisher
        self.config = config
        self.down_keys = set()
        self.should_update_twist = True
        
        # Speed settings
        self.linear_speed = config['robot']['linear_speed']
        self.angular_speed = config['robot']['angular_speed']
        
        # PTZ settings
        self.pan = 0
        self.tilt = 0
        self.zoom = 0
        self.prev_pan = 0
        self.prev_tilt = 0
        self.prev_zoom = 0
        self.ptz_lock = threading.Lock()
        
    def get_linear_speed(self):
        """Return current linear speed"""
        return self.linear_speed
    
    def handle_keydown(self, key, socketio):
        """Handle keydown event"""
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
                'linear': self.linear_speed, 
                'angular': self.angular_speed
            })
            print(f"Updated speeds: Linear={self.linear_speed}, Angular={self.angular_speed}")
    
    def handle_keyup(self, key):
        """Handle keyup event"""
        print(f"Key up: {key}")
        self.down_keys.discard(key)
        twist = Twist()
        self.pub.publish(twist)
    
    def clear_keys_and_stop(self):
        """Clear all keys and stop robot"""
        self.down_keys.clear()
        self.should_update_twist = False
        twist = Twist()
        self.pub.publish(twist)
    
    def update_ptz(self):
        """Update PTZ camera"""
        updated = False
        with self.ptz_lock:
            if 'w' in self.down_keys:
                self.tilt = min(
                    self.tilt + self.config['ptz']['tilt_step'], 
                    self.config['ptz']['tilt_max']
                )
                updated = True
            if 's' in self.down_keys:
                self.tilt = max(
                    self.tilt - self.config['ptz']['tilt_step'], 
                    self.config['ptz']['tilt_min']
                )
                updated = True
            if 'a' in self.down_keys:
                self.pan = min(
                    self.pan + self.config['ptz']['pan_step'], 
                    self.config['ptz']['pan_max']
                )
                updated = True
            if 'd' in self.down_keys:
                self.pan = max(
                    self.pan - self.config['ptz']['pan_step'], 
                    self.config['ptz']['pan_min']
                )
                updated = True
            if 'x' in self.down_keys:
                self.zoom = min(
                    self.zoom + self.config['ptz']['zoom_step'], 
                    self.config['ptz']['zoom_max']
                )
                updated = True
            if 'z' in self.down_keys:
                self.zoom = max(
                    self.zoom - self.config['ptz']['zoom_step'], 
                    self.config['ptz']['zoom_min']
                )
                updated = True
            
            if updated and (self.pan != self.prev_pan or 
                          self.tilt != self.prev_tilt or 
                          self.zoom != self.prev_zoom):
                cmd = (f"v4l2-ctl --set-ctrl=pan_absolute={self.pan} "
                      f"--set-ctrl=tilt_absolute={self.tilt} "
                      f"--set-ctrl=zoom_absolute={self.zoom}")
                os.system(cmd)
                self.prev_pan, self.prev_tilt, self.prev_zoom = self.pan, self.tilt, self.zoom
    
    def update_loop(self, auto_mode_checker):
        """Robot control update loop (runs in separate thread)"""
        while True:
            # Skip manual control during autonomous mode
            if auto_mode_checker():
                time.sleep(0.1)
                continue
                
            if not self.should_update_twist:
                time.sleep(0.1)
                continue
            
            # Update PTZ
            self.update_ptz()
            
            # Control robot movement
            twist = Twist()
            if 'ArrowUp' in self.down_keys:
                twist.linear.x += self.linear_speed
            if 'ArrowDown' in self.down_keys:
                twist.linear.x -= self.linear_speed
            if 'ArrowLeft' in self.down_keys:
                twist.angular.z += self.angular_speed
            if 'ArrowRight' in self.down_keys:
                twist.angular.z -= self.angular_speed
            
            self.pub.publish(twist)
            time.sleep(0.1)