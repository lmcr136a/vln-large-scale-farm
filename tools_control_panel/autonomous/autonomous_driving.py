"""
Autonomous driving — closed-loop path following in the map frame.
Yields one command dict per CONTROL_DT for the caller to publish.

Control strategy:
  - Pure-pursuit look-ahead on the path segment keeps cross-track error ≤ cross_track_limit.
  - In the translate phase, vt and vr are issued simultaneously (proportional steering).
  - Stop-and-rotate only when |angle_error| exceeds stop_rot_r = 2 × ori_tol.
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


def _angle_diff(a, b):
    """Signed shortest-path difference (a − b) in (−π, π]."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def run(waypoints, get_robot_pose, params=None):
    """
    Generator yielding command dicts:
      {'vt', 'vr', 'dt', 'status', 'completed', 'waypoint_reached'(optional)}
    """
    p          = params or {}
    pos_tol    = p.get('position_tolerance', 0.3)
    ori_tol_r  = math.radians(p.get('orientation_tolerance', 8))
    # Errors below ori_tol_r  → translate + proportional vr (continuous correction).
    # Errors above stop_rot_r → stop and rotate in place.
    # Between the two         → still translate + vr, capped at max_simul_vr.
    stop_rot_r = math.radians(p.get('stop_rotate_threshold', 15))
    cte_max    = p.get('cross_track_limit',  0.5)   # m — max allowed deviation from path line
    lookahead  = p.get('lookahead_distance', 1.5)   # m — base look-ahead distance

    r_stages = _load_stages(p, 'r', DEFAULT_R_STAGES)
    t_stages = _load_stages(p, 't', DEFAULT_T_STAGES)

    # Angular P-gain for simultaneous steering: full max_simul_vr at ori_tol_r
    max_simul_vr = r_stages[1][0] if len(r_stages) > 1 else r_stages[0][0]

    wp_idx  = 0
    n_wp    = len(waypoints)
    prev_xy = None   # start of current path leg (set to robot pos on first iteration)

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

            # Record leg start on the very first valid pose
            if prev_xy is None:
                prev_xy = (rx, ry)

            dx   = tx - rx
            dy   = ty - ry
            dist = math.hypot(dx, dy)

            if dist < pos_tol:
                yield {'vt': 0.0, 'vr': 0.0, 'dt': 0.0,
                       'waypoint_reached': wp_idx,
                       'completed': False,
                       'status': f'Reached waypoint {wp_idx + 1}/{n_wp}'}
                prev_xy = (tx, ty)   # this waypoint becomes the start of the next leg
                wp_idx += 1
                break

            # ── Pure-pursuit: look-ahead point on the path line ───────────────
            prev_x, prev_y = prev_xy
            path_dx = tx - prev_x
            path_dy = ty - prev_y
            path_len = math.hypot(path_dx, path_dy)

            if path_len > 0.1:
                # Scalar projection of robot onto [prev_xy → wp] segment
                t_proj = ((rx - prev_x) * path_dx + (ry - prev_y) * path_dy) / (path_len ** 2)
                t_proj = max(0.0, min(1.0, t_proj))

                # Cross-track error (perpendicular distance from robot to path)
                clos_x = prev_x + t_proj * path_dx
                clos_y = prev_y + t_proj * path_dy
                cte    = math.hypot(rx - clos_x, ry - clos_y)

                # Look-ahead shrinks when cte is large → stronger pull back to path
                la   = max(0.3, lookahead * max(0.0, 1.0 - cte / cte_max))
                la_t = min(1.0, t_proj + la / path_len)
                la_x = prev_x + la_t * path_dx
                la_y = prev_y + la_t * path_dy
                target_yaw = math.atan2(la_y - ry, la_x - rx)
            else:
                cte        = 0.0
                target_yaw = math.atan2(dy, dx)

            angle_err = _angle_diff(target_yaw, yaw)
            abs_err   = abs(angle_err)

            # ── Phase transitions ─────────────────────────────────────────────
            if phase == 'translate' and abs_err > stop_rot_r:
                phase = 'rotate'
            if phase == 'rotate'   and abs_err <= stop_rot_r:
                phase = 'translate'

            if phase == 'rotate':
                # Stop and rotate in place
                vr, stage = _pick_vel(math.degrees(abs_err), r_stages)
                vr  *= 1 if angle_err > 0 else -1
                side = 'Left' if angle_err > 0 else 'Right'
                yield {'vt': 0.0, 'vr': vr, 'dt': CONTROL_DT,
                       'completed': False,
                       'status': f'Rotating {side}: {math.degrees(abs_err):.1f}°'}

            else:
                # Simultaneous forward + proportional angular correction
                vt, stage = _pick_vel(dist, t_stages)

                # Proportional vr: full max_simul_vr at ori_tol_r, tapers to 0 as error → 0
                vr = (angle_err / ori_tol_r) * max_simul_vr
                vr = max(-max_simul_vr, min(max_simul_vr, vr))

                # Slow down proportionally to angle error so steering can keep up
                vt *= max(0.5, 1.0 - abs_err / stop_rot_r)

                cte_str = f'  cte={cte:.2f}m' if cte > 0.05 else ''
                yield {'vt': vt, 'vr': vr, 'dt': CONTROL_DT,
                       'completed': False,
                       'status': (f'Moving: {dist:.2f}m  err={math.degrees(abs_err):.1f}°'
                                  f'{cte_str}  Speed={stage}')}

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
            angle_err = _angle_diff(target_yaw, pose['yaw'])
            if abs(angle_err) <= ori_tol_r:
                break
            vr, stage = _pick_vel(math.degrees(abs(angle_err)), r_stages)
            vr  *= 1 if angle_err > 0 else -1
            side = 'Left' if angle_err > 0 else 'Right'
            yield {'vt': 0.0, 'vr': vr, 'dt': CONTROL_DT,
                   'completed': False,
                   'status': f'Final orientation {side}: {math.degrees(abs(angle_err)):.1f}°'}

    yield {'vt': 0.0, 'vr': 0.0, 'dt': 0.0, 'completed': True, 'status': ''}
