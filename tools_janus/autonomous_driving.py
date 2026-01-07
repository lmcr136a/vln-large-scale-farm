#!/usr/bin/env python3
"""
Autonomous driving module for path following
"""

import math
import time
import sys
import threading

def run(waypoints, get_robot_pose):
    vt = 0.2
    vr = 0.3
    
    for i, (target_x, target_y) in enumerate(waypoints):
        
        while True:
            robot_x, robot_y, robot_yaw = get_robot_pose()
            
            dx = target_x - robot_x
            dy = target_y - robot_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 0.3:
                yield {'vt': 0.0, 'vr': 0.0, 'tt': 0.0, 'tr': 0.0, 'completed': False, 'waypoint_reached': i}
                break
            
            target_yaw = -math.atan2(dy, dx)
            angle_diff = target_yaw - robot_yaw
            
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            if abs(angle_diff) > math.radians(10):       
                angle_ref1, angle_ref2, angle_ref3 = 15, 60, 120         
                if angle_diff > 0:
                    print("Turning Left")
                    if angle_diff > math.radians(angle_ref3):
                        yield {'vt': 0.0, 'vr': vr, 'tt': 0.0, 'tr': 4.0, 'completed': False, 'status': 'Turning Left'}
                    elif angle_diff > math.radians(angle_ref2):
                        yield {'vt': 0.0, 'vr': vr, 'tt': 0.0, 'tr': 3.0, 'completed': False, 'status': 'Turning Left'}
                    elif angle_diff > math.radians(angle_ref1):
                        yield {'vt': 0.0, 'vr': vr, 'tt': 0.0, 'tr': 2.0, 'completed': False, 'status': 'Turning Left'}
                    else:
                        yield {'vt': 0.0, 'vr': vr, 'tt': 0.0, 'tr': 1.0, 'completed': False, 'status': 'Turning Left'}
                else:
                    print("Turning Right")
                    if angle_diff < math.radians(-angle_ref3):
                        yield {'vt': 0.0, 'vr': -vr, 'tt': 0.0, 'tr': 4.0, 'completed': False, 'status': 'Turning Right'}
                    elif angle_diff < math.radians(-angle_ref2):
                        yield {'vt': 0.0, 'vr': -vr, 'tt': 0.0, 'tr': 3.0, 'completed': False, 'status': 'Turning Right'}
                    elif angle_diff < math.radians(-angle_ref1):
                        yield {'vt': 0.0, 'vr': -vr, 'tt': 0.0, 'tr': 2.0, 'completed': False, 'status': 'Turning Right'}
                    else:
                        yield {'vt': 0.0, 'vr': -vr, 'tt': 0.0, 'tr': 1.0, 'completed': False, 'status': 'Turning Right'}
            else:
                print("Going Forward")
                if distance > 3:
                    yield {'vt': vt, 'vr': 0.0, 'tt': 4.0, 'tr': 0.0, 'completed': False, 'status': 'Going Forward'}
                elif distance > 1.5:
                    yield {'vt': vt, 'vr': 0.0, 'tt': 2.0, 'tr': 0.0, 'completed': False, 'status': 'Going Forward'}
                else:
                    yield {'vt': vt, 'vr': 0.0, 'tt': 1.0, 'tr': 0.0, 'completed': False, 'status': 'Going Forward'}
    
    yield {'vt': 0.0, 'vr': 0.0, 'tt': 0.0, 'tr': 0.0, 'completed': True, 'status': ''}