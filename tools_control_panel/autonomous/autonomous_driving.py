"""
Autonomous driving — closed-loop path following in the map frame.
Yields one command dict per CONTROL_DT for the caller to publish.

Control strategy:
  - Cross-track regulator steers to hold the path LINE itself: target heading is the
    path direction plus a correction angle that grows with the perpendicular offset
    (saturates at ±90°) and decays to 0 on the line → converges parallel, no overshoot.
  - In the translate phase, vt and vr are issued simultaneously (proportional steering).
  - Stop-and-rotate only when |angle_error| exceeds stop_rot_r = 2 × ori_tol.
"""
import math

CONTROL_DT = 0.1

# Strength of the continuous heading-correction while translating. The angular
# velocity is proportional to (target_yaw − yaw); this gain multiplies that
# response. 2.0 = twice as aggressive as the plain proportional term.
DIRECTION_CORRECTION_GAIN = 1.0

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


def run(waypoints, get_robot_pose, params=None, start_index=0):
    """
    Generator yielding command dicts:
      {'vt', 'vr', 'dt', 'status', 'completed', 'waypoint_reached'(optional)}

    start_index: index of the first waypoint to head for. Used by "Resume" to
    continue from the nearest waypoint after a mid-drive stop instead of the
    first one. Defaults to 0 (drive the whole path from the start).
    """
    p          = params or {}
    pos_tol    = p.get('position_tolerance', 0.3)
    ori_tol_r  = math.radians(p.get('orientation_tolerance', 8))
    # Errors below ori_tol_r  → translate + proportional vr (continuous correction).
    # Errors above stop_rot_r → stop and rotate in place.
    # Between the two         → still translate + vr, capped at max_simul_vr.
    stop_rot_r = math.radians(p.get('stop_rotate_threshold', 15))
    cte_max    = p.get('cross_track_limit',  0.5)   # m — max allowed deviation from path line
    # Cross-track regulator: steer to hold the path LINE itself (no look-ahead point).
    cte_gain   = p.get('cross_track_gain', 1.0)        # higher → cuts back to the line harder
    cte_soft   = p.get('cross_track_softening', 0.4)   # m — correction reaches ~45° at this offset
    # Heading-correction strength (1.0 = plain proportional, 2.0 = twice as aggressive).
    dir_gain   = p.get('direction_correction_gain', DIRECTION_CORRECTION_GAIN)

    r_stages = _load_stages(p, 'r', DEFAULT_R_STAGES)
    t_stages = _load_stages(p, 't', DEFAULT_T_STAGES)

    # Angular P-gain for simultaneous steering: full max_simul_vr at ori_tol_r
    max_simul_vr = r_stages[1][0] if len(r_stages) > 1 else r_stages[0][0]

    n_wp    = len(waypoints)
    wp_idx  = max(0, min(start_index, n_wp - 1)) if n_wp else 0
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

            # ── Cross-track regulator: hold the path LINE (no look-ahead point) ─
            prev_x, prev_y = prev_xy
            path_dx = tx - prev_x
            path_dy = ty - prev_y
            path_len = math.hypot(path_dx, path_dy)

            if path_len > 0.1:
                path_yaw = math.atan2(path_dy, path_dx)
                # Signed cross-track error: >0 when robot is LEFT of the path
                # direction, <0 when RIGHT. Magnitude = perpendicular distance.
                cross = (path_dx * (ry - prev_y) - path_dy * (rx - prev_x)) / path_len
                cte   = abs(cross)

                # Steer back toward the line: the correction angle grows with the
                # offset and saturates at ±90° (heads straight at the line when far
                # off), then decays to 0 as the robot reaches the line → ends up
                # parallel with no overshoot. cte_gain/cte_soft tune the stiffness.
                corr = math.atan2(cte_gain * cross, cte_soft)
                target_yaw = path_yaw - corr
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

                # Proportional vr: full max_simul_vr at ori_tol_r, tapers to 0 as error → 0.
                # DIRECTION_CORRECTION_GAIN makes the heading correction stronger so the
                # robot snaps back onto the target heading faster (saturates sooner).
                vr = (angle_err / ori_tol_r) * max_simul_vr * dir_gain
                vr = max(-max_simul_vr, min(max_simul_vr, vr))

                # Slow down proportionally to angle error so steering can keep up
                vt *= max(0.5, 1.0 - abs_err / stop_rot_r)

                # ── Hard cross-track safety: never let the robot leave the line by
                # more than cte_max. Forward speed tapers from full (at half the
                # limit) down to 0 (at the limit), so it stops advancing and only
                # steers back to the line instead of drifting further out.
                if cte_max > 0:
                    vt *= max(0.0, min(1.0, (cte_max - cte) / (0.5 * cte_max)))

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
