#!/usr/bin/env python3
"""
Autonomous driving module for path following.

Stage config format (in control_config.yaml):
  r1: [time, vel, range_deg]   # rotation stages, range = upper bound in degrees
  r2: [time, vel, range_deg]   #   r1 = smallest angle, r3 = largest angle
  r3: [time, vel, range_deg]

  t1: [time, vel, range_m]     # translation stages, range = upper bound in meters
  t2: [time, vel, range_m]     #   t1 = closest, t3 = farthest
  t3: [time, vel, range_m]

  Stages are checked ascending by range; the first stage where remaining <= range is used.
  If remaining exceeds all ranges, the last (highest) stage is used.
"""

import math


# ── Default stages (used if config is missing) ─────────────────────────────
DEFAULT_R_STAGES = [
    (1.0, 0.3, 15),    # a1: tr=1s, vr=0.3 rad/s, up to 15 deg
    (2.0, 0.5, 60),    # a2: tr=2s, vr=0.5 rad/s, up to 60 deg
    (3.0, 0.7, 360),   # a3: tr=3s, vr=0.7 rad/s, above 60 deg
]

DEFAULT_T_STAGES = [
    (1.0, 0.3, 1.5),   # t1: tt=1s, vt=0.3 m/s, up to 1.5m
    (2.0, 0.5, 3.0),   # t2: tt=2s, vt=0.5 m/s, up to 3.0m
    (4.0, 0.8, 999),   # t3: tt=4s, vt=0.8 m/s, above 3.0m
]


def _load_stages(p, prefix, defaults):
    """
    Load r1/r2/r3 or t1/t2/t3 from params dict.
    Each entry is [time, vel, range]. Returns list of (time, vel, range) sorted by range asc.
    """
    stages = []
    for k, default in zip(['1', '2', '3'], defaults):
        raw = p.get(f'{prefix}{k}', default)
        stages.append(tuple(raw))
    return sorted(stages, key=lambda s: s[2])


def _pick_stage(remaining, stages):
    """
    Return (stage_idx 1-based, time, vel).
    First stage where remaining <= range; falls back to last stage.
    """
    for i, (t, v, r) in enumerate(stages):
        if remaining <= r:
            return i + 1, t, v
    last = stages[-1]
    return len(stages), last[0], last[1]


def _pct(remaining, initial):
    if initial <= 0:
        return 0
    return max(0, min(100, int((1.0 - remaining / initial) * 100)))


def run(waypoints, get_robot_pose, params=None):
    p = params or {}

    position_tolerance    = p.get('position_tolerance',    0.3)
    orientation_tolerance = p.get('orientation_tolerance', 10)

    r_stages = _load_stages(p, 'r', DEFAULT_R_STAGES)
    t_stages = _load_stages(p, 't', DEFAULT_T_STAGES)

    for i, waypoint in enumerate(waypoints):
        target_x = waypoint['x']
        target_y = waypoint['y']

        # Capture initial distance for progress percentage
        robot_x, robot_y, _ = get_robot_pose()
        dx0, dy0 = target_x - robot_x, target_y - robot_y
        initial_dist  = math.sqrt(dx0*dx0 + dy0*dy0)
        initial_angle = None   # latched on first rotation step

        while True:
            robot_x, robot_y, robot_yaw = get_robot_pose()

            dx = target_x - robot_x
            dy = target_y - robot_y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance < position_tolerance:
                yield {'vt': 0.0, 'vr': 0.0, 'tt': 0.0, 'tr': 0.0,
                       'completed': False, 'waypoint_reached': i}
                break

            target_yaw = -math.atan2(dy, dx)
            angle_diff = target_yaw - robot_yaw

            while angle_diff >  math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi

            abs_angle = abs(angle_diff)

            if abs_angle > math.radians(orientation_tolerance):
                # ── Rotation phase ──────────────────────────────────────
                if initial_angle is None:
                    initial_angle = abs_angle

                stage_idx, tr, vr = _pick_stage(math.degrees(abs_angle), r_stages)
                pct = _pct(abs_angle, initial_angle)
                direction = 'Left ' if angle_diff > 0 else 'Right'
                print(f"Turning {direction}  r{stage_idx} - {pct}% | {math.degrees(abs_angle):.1f}°  "
                      f"[tr={tr}s  vr={vr}]")

                vr_signed = vr if angle_diff > 0 else -vr
                status = f'Turning {"Left" if angle_diff > 0 else "Right"}'
                yield {'vt': 0.0, 'vr': vr_signed, 'tt': 0.0, 'tr': tr,
                       'completed': False, 'status': status}

            else:
                # ── Translation phase ────────────────────────────────────
                initial_angle = None   # reset for next rotation

                stage_idx, tt, vt = _pick_stage(distance, t_stages)
                pct = _pct(distance, initial_dist)
                print(f"Going Forward   t{stage_idx} - {pct}% | {distance:.1f}m  "
                      f"[tt={tt}s  vt={vt}]")

                yield {'vt': vt, 'vr': 0.0, 'tt': tt, 'tr': 0.0,
                       'completed': False, 'status': 'Going Forward'}

    yield {'vt': 0.0, 'vr': 0.0, 'tt': 0.0, 'tr': 0.0, 'completed': True, 'status': ''}