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
        self.pub = publisher
        self.socketio = socketio
        self.config = config
        
        self.auto_mode_active = False
        self.auto_thread = None
        self.auto_stop_event = threading.Event()
        
    def is_active(self):
        """Return autonomous mode active status"""
        return self.auto_mode_active
    
    def get_robot_pose(self, default_x, default_y, default_yaw):
        """Get current robot position"""
        map_dir = os.path.expanduser(self.config['paths']['map_dir'])
        map_yaml = os.path.join(map_dir, 'map_latest.yaml')
        
        if os.path.exists(map_yaml):
            with open(map_yaml, 'r') as f:
                yaml_data = yaml.safe_load(f)
                x = yaml_data.get('robot_x', default_x)
                y = yaml_data.get('robot_y', default_y)
                yaw = yaml_data.get('robot_yaw', default_yaw)
                return x, y, yaw
        
        return default_x, default_y, default_yaw
    
    def autonomous_control_loop(self, robot_x, robot_y, robot_yaw, waypoints):
        """Autonomous control loop"""
        max_repeat = self.config['autonomous']['max_repeat_num']
        
        self.socketio.emit('robot_status', {
            'status': f'Navigating to {len(waypoints)} waypoints'
        }, namespace='/')
        
        print(f"Starting autonomous driving from ({robot_x:.2f}, {robot_y:.2f})")
        print(f"Waypoints: {waypoints}")
        
        repeat_count = 0
        try:
            while repeat_count < max_repeat:
                # Run autonomous_driving module
                for control_cmd in autonomous_driving.run(
                    waypoints, 
                    lambda: self.get_robot_pose(robot_x, robot_y, robot_yaw)
                ):
                    # Check stop signal
                    if self.auto_stop_event.is_set():
                        print("Autonomous driving stopped by user")
                        self.socketio.emit('robot_status', {
                            'status': 'Stopped by user'
                        }, namespace='/')
                        break
                    
                    # Execute control command
                    vt = control_cmd.get('vt', 0.0)
                    vr = control_cmd.get('vr', 0.0)
                    tt = control_cmd.get('tt', 0.0)
                    tr = control_cmd.get('tr', 0.0)
                    wp_idx = control_cmd.get('waypoint_index', -1)
                    
                    # Publish Twist message
                    twist = Twist()
                    twist.linear.x = vt
                    twist.angular.z = vr
                    self.pub.publish(twist)
                    
                    # Notify waypoint reached
                    if wp_idx >= 0:
                        print(f"Reached waypoint {wp_idx + 1}/{len(waypoints)}")
                        self.socketio.emit('waypoint_reached', {
                            'index': wp_idx
                        }, namespace='/')
                    
                    time.sleep(tt + tr)
                
                # Exit loop if stopped
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
            # Stop robot
            twist = Twist()
            self.pub.publish(twist)
            
            self.auto_mode_active = False
            self.socketio.emit('auto_mode_completed', namespace='/')
            self.socketio.emit('robot_status', {'status': ''}, namespace='/')
            print("Autonomous driving completed")
    
    def start(self, robot_x, robot_y, robot_yaw, waypoints):
        """Start autonomous driving"""
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
        """Stop autonomous driving"""
        if not self.auto_mode_active:
            return False
        
        print("Stopping autonomous mode")
        self.auto_stop_event.set()
        self.auto_mode_active = False
        
        # Stop robot
        twist = Twist()
        self.pub.publish(twist)
        
        return True