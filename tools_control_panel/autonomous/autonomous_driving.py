"""
Autonomous driving — closed-loop path following in the map frame.
Yields one command dict per CONTROL_DT for the caller to publish.
"""
import math

CONTROL_DT = 0.1

DEFAULT_R_STAGES = [(0.3, 15), (0.5, 60), (0.7, 360)]
DEFAULT_T_STAGES = [(0.3, 1.5), (0.5, 3.0), (0.8, 999)]


def _load_stages(params, prefix, defaults):
    stages = []
    for k, default in zip(['1', '2', '3'], defaults):
        stages.append(tuple(params.get(f'{prefix}{k}', default)))
    return sorted(stages, key=lambda s: s[1])


def _pick_vel(remaining, stages):
    for i, (vel, rng) in enumerate(stages):
        if remaining <= rng:
            return vel, i + 1
    return stages[-1][0], len(stages)


def run(waypoints, get_robot_pose, params=None):
    """
    Generator yielding command dicts:
      {'vt', 'vr', 'dt', 'status', 'completed', 'waypoint_reached'(optional)}
    """
    p         = params or {}
    pos_tol   = p.get('position_tolerance', 0.3)
    ori_tol_r = math.radians(p.get('orientation_tolerance', 8))
    re_rot_tol = ori_tol_r * 2

    r_stages = _load_stages(p, 'r', DEFAULT_R_STAGES)
    t_stages = _load_stages(p, 't', DEFAULT_T_STAGES)

    wp_idx = 0
    n_wp   = len(waypoints)

    while wp_idx < n_wp:
        wp     = waypoints[wp_idx]
        tx, ty = wp['x'], wp['y']
        phase  = 'rotate'

        while True:
            pose = get_robot_pose()
            if pose is None:
                yield {'vt': 0.0, 'vr': 0.0, 'dt': CONTROL_DT,
                       'completed': False, 'status': 'Waiting for pose'}
                continue

            rx, ry, yaw = pose['x'], pose['y'], pose['yaw']
            dx   = tx - rx
            dy   = ty - ry
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < pos_tol:
                yield {'vt': 0.0, 'vr': 0.0, 'dt': 0.0,
                       'waypoint_reached': wp_idx,
                       'completed': False,
                       'status': f'Reached waypoint {wp_idx + 1}/{n_wp}'}
                wp_idx += 1
                break

            target_yaw = math.atan2(dy, dx)
            angle_err  = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
            abs_err    = abs(angle_err)

            if phase == 'translate' and abs_err > re_rot_tol:
                phase = 'rotate'
            if phase == 'rotate' and abs_err <= ori_tol_r:
                phase = 'translate'

            if phase == 'rotate':
                vr, stage = _pick_vel(math.degrees(abs_err), r_stages)
                vr *= 1 if angle_err > 0 else -1
                side = 'Left' if angle_err > 0 else 'Right'
                yield {'vt': 0.0, 'vr': vr, 'dt': CONTROL_DT,
                       'completed': False,
                       'status': f'Rotating {side}: {math.degrees(abs_err):.1f}° Speed={stage}'}
            else:
                vt, stage = _pick_vel(dist, t_stages)
                yield {'vt': vt, 'vr': 0.0, 'dt': CONTROL_DT,
                       'completed': False,
                       'status': f'Moving Forward: {dist:.2f}m Speed={stage}'}

    # Final orientation — rotate to target yaw if last waypoint has one
    last_wp = waypoints[-1] if waypoints else None
    if last_wp and 'yaw' in last_wp:
        target_yaw = float(last_wp['yaw'])
        while True:
            pose = get_robot_pose()
            if pose is None:
                yield {'vt': 0.0, 'vr': 0.0, 'dt': CONTROL_DT,
                       'completed': False, 'status': 'Waiting for pose'}
                continue
            angle_err = (target_yaw - pose['yaw'] + math.pi) % (2 * math.pi) - math.pi
            if abs(angle_err) <= ori_tol_r:
                break
            vr, stage = _pick_vel(math.degrees(abs(angle_err)), r_stages)
            vr *= 1 if angle_err > 0 else -1
            side = 'Left' if angle_err > 0 else 'Right'
            yield {'vt': 0.0, 'vr': vr, 'dt': CONTROL_DT,
                   'completed': False,
                   'status': f'Final orientation {side}: {math.degrees(abs(angle_err)):.1f}°'}

    yield {'vt': 0.0, 'vr': 0.0, 'dt': 0.0, 'completed': True, 'status': ''}