#!/usr/bin/env python3
"""
Autonomous driving — closed-loop path following in the 3D map frame.

get_robot_pose() returns {x, y, z, yaw} directly from the mapper node
(updated at LiDAR frequency, ~10 Hz).  Navigation uses XY distance only;
Z is available but ignored for waypoint targeting.

Control loop runs at CONTROL_DT seconds.  Every iteration:
  1. Fetch current pose from mapper
  2. Compute XY error to current waypoint
  3. If within position_tolerance → mark reached, advance to next waypoint
  4. If heading error > orientation_tolerance → rotate
  5. Else → translate
  6. Yield a single (vt, vr, dt) command; autonomous_mode.py publishes it

Stage config (control_config.yaml, under 'autonomous'):
  r1: [vel, range_deg]   # rotate slow   up to range_deg degrees
  r2: [vel, range_deg]   # rotate medium
  r3: [vel, range_deg]   # rotate fast   (used above r2 range)
  t1: [vel, range_m]     # translate slow   up to range_m
  t2: [vel, range_m]     # translate medium
  t3: [vel, range_m]     # translate fast   (used above t2 range)
"""

import math

CONTROL_DT = 0.1   # control loop period in seconds

DEFAULT_R_STAGES = [
    (0.3, 15),    # up to 15 deg: slow
    (0.5, 60),    # up to 60 deg: medium
    (0.7, 360),   # above 60 deg: fast
]

DEFAULT_T_STAGES = [
    (0.3, 1.5),   # up to 1.5 m: slow
    (0.5, 3.0),   # up to 3.0 m: medium
    (0.8, 999),   # above 3.0 m: fast
]


def _load_stages(params, prefix, defaults):
    """Load r1/r2/r3 or t1/t2/t3 from params dict. Returns list of (vel, range) sorted by range."""
    stages = []
    for k, default in zip(['1', '2', '3'], defaults):
        raw = params.get(f'{prefix}{k}', default)
        stages.append(tuple(raw))
    return sorted(stages, key=lambda s: s[1])


def _pick_vel(remaining, stages):
    """Return velocity for the first stage whose range >= remaining."""
    for vel, rng in stages:
        if remaining <= rng:
            return vel
    return stages[-1][0]


def run(waypoints, get_robot_pose, params=None):
    """
    Closed-loop path following generator.

    Yields one dict per CONTROL_DT:
      {'vt': float, 'vr': float, 'dt': float,
       'waypoint_reached': int (optional),
       'completed': bool, 'status': str}

    Caller is responsible for publishing Twist(vt, vr) for dt seconds,
    then calling next() immediately to re-check pose.
    """
    p = params or {}

    pos_tol    = p.get('position_tolerance',    0.3)     # metres
    ori_tol    = p.get('orientation_tolerance', 8)       # degrees
    ori_tol_r  = math.radians(ori_tol)
    re_rot_tol = ori_tol_r * 2                           # heading drift threshold during translation

    r_stages = _load_stages(p, 'r', DEFAULT_R_STAGES)
    t_stages = _load_stages(p, 't', DEFAULT_T_STAGES)

    waypoint_idx = 0
    n_wp = len(waypoints)

    while waypoint_idx < n_wp:
        wp = waypoints[waypoint_idx]
        tx, ty = wp['x'], wp['y']   # target XY in map frame

        print(f'[WP {waypoint_idx+1}/{n_wp}] target ({tx:.2f}, {ty:.2f})')
        phase = 'rotate'   # start each waypoint with a rotation phase

        while True:
            pose = get_robot_pose()
            if pose is None:
                # No pose yet — wait
                yield {'vt': 0.0, 'vr': 0.0, 'dt': CONTROL_DT,
                       'completed': False, 'status': 'Waiting for pose'}
                continue

            rx, ry, yaw = pose['x'], pose['y'], pose['yaw']
            dx       = tx - rx
            dy       = ty - ry
            dist     = math.sqrt(dx*dx + dy*dy)

            # --- Waypoint reached? ---
            if dist < pos_tol:
                print(f'  Reached WP {waypoint_idx+1}')
                yield {
                    'vt': 0.0, 'vr': 0.0, 'dt': 0.0,
                    'waypoint_reached': waypoint_idx,
                    'completed': False, 'status': 'Waypoint reached',
                }
                waypoint_idx += 1
                break

            # Desired heading toward target (map frame, CCW positive)
            target_yaw = math.atan2(dy, dx)
            angle_err  = target_yaw - yaw
            # Wrap to [-pi, pi]
            angle_err  = (angle_err + math.pi) % (2 * math.pi) - math.pi
            abs_err    = abs(angle_err)

            # --- Phase transitions ---
            if phase == 'translate' and abs_err > re_rot_tol:
                print(f'  Heading drift {math.degrees(abs_err):.1f}° — re-rotating')
                phase = 'rotate'

            if phase == 'rotate' and abs_err <= ori_tol_r:
                phase = 'translate'

            # --- Commands ---
            if phase == 'rotate':
                vr = _pick_vel(math.degrees(abs_err), r_stages)
                vr = vr if angle_err > 0 else -vr
                direction = 'Left' if angle_err > 0 else 'Right'
                print(f'  Rotate {direction}: {math.degrees(abs_err):.1f}°  vr={vr:.2f}')
                yield {
                    'vt': 0.0, 'vr': vr, 'dt': CONTROL_DT,
                    'completed': False,
                    'status': f'Turning {direction}',
                }
            else:
                vt = _pick_vel(dist, t_stages)
                print(f'  Forward: {dist:.2f} m  vt={vt:.2f}')
                yield {
                    'vt': vt, 'vr': 0.0, 'dt': CONTROL_DT,
                    'completed': False,
                    'status': 'Going Forward',
                }

    # All waypoints done
    yield {'vt': 0.0, 'vr': 0.0, 'dt': 0.0, 'completed': True, 'status': ''}