"""
GPS-based top-down map generator.

Generates a North-up PNG image + map_state.json in the format the web panel
expects (control.js / path_plan.js worldToPixel / pixelToWorld).

map_state fields:
  resolution  — metres per pixel
  origin_x    — ENU x at image left edge
  origin_y    — ENU y at image bottom
  img_width   — pixels
  img_height  — pixels
  rot_angle   — always 0.0 (North-up); kept for web-panel compatibility.

When a crop_field.json is available, the field's south edge (ref_x, y_south)
is treated as a fixed, permanent survey baseline. rotation_deg tilts only the
north end of each row sideways (a shear, not a rotation), so the south
corners never move when rotation_deg is adjusted — the field renders as a
parallelogram. The image is cropped tightly to this shape + CROP_MARGIN,
expanded as needed so visible track/landmarks are never clipped.
Without a crop_field.json, the map falls back to a North-up view sized from
the visible track/landmarks (legacy behaviour, for use before the field is
calibrated).
"""
import json
import logging
import math
import os
import threading
import time

import numpy as np

log = logging.getLogger(__name__)

RESOLUTION   = 0.05   # metres per pixel (5 cm/px)
GRID_SPACING = 10.0   # metres between grid lines
MAP_INTERVAL = 3.0    # seconds between regeneration
CROP_MARGIN  = 10.0   # metres of blank margin kept around the crop field
MAX_IMG_PX   = 6000   # hard cap per image dimension

ROBOT_HALF_M = 0.3    # robot footprint is 0.6 x 0.6 m (half-width = 0.3 m)

# TEMP (user-requested): pause landmark drawing while accuracy work continues.
LANDMARKS_PAUSED = True

# RGB colours
C_BG       = (255, 255, 255)   # white background
C_GRID     = (220, 220, 220)   # light gray grid lines
C_TRAIL    = ( 80, 160,  80)
C_ROBOT    = (255, 100,   0)
C_WAYPOINT = ( 60, 140, 220)

TYPE_COLOURS = {
    'metal_box':  (200, 160, 130),
    'trailer':    (200, 160, 130),
    'vehicle':    (200, 160, 130),
    'building':   (200, 160, 130),
    'crop_stick': (200, 150,   0),
}
C_DEFAULT_LM = (200, 160, 130)


def _gps_to_enu(lat, lon, origin_lat, origin_lon):
    R = 6378137.0
    x = R * math.radians(lon - origin_lon) * math.cos(math.radians(origin_lat))
    y = R * math.radians(lat - origin_lat)
    return x, y


# Field stripe constants
_FT = 0.3048            # 1 foot in metres
C_FIELD_GREEN = (180, 220, 200)
STRIPE_GREEN_W = 10 * _FT  # 3.048 m
STRIPE_WHITE_W = 4 * _FT   # 1.2192 m
MAX_GREEN_STRIPES = 15


def _load_crop(crop_file: str | None) -> dict | None:
    """Load crop_field.json → {ref_x, y_south, y_north, rotation_deg} or None."""
    if not crop_file:
        return None
    try:
        with open(crop_file) as f:
            d = json.load(f)
        if 'ref_x' in d and 'y_south' in d and 'y_north' in d:
            d.setdefault('rotation_deg', 0.0)
            return d
    except Exception:
        pass
    return None


def generate_map(localizer, store, output_dir: str,
                 robot_pose: dict | None = None,
                 waypoints: list | None = None,
                 past_paths: list | None = None,
                 crop_file: str | None = None) -> dict:
    """
    Generate top-down PNG.
    crop_file: path to crop_field.json with {ref_x, y_south, y_north, rotation_deg}.
               When present, the image is cropped tightly to this field's shape
               (+ CROP_MARGIN) and aligned to its orientation. Stripes are drawn
               permanently from that file — never from mission points.
    Returns map_state dict (empty dict on failure).
    """
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        log.warning('Pillow not installed — pip install Pillow')
        return {}

    origin_lat, origin_lon = localizer.get_origin()
    if origin_lat is None:
        return {}

    track     = localizer.get_track()
    landmarks = store.get_all()
    crop      = _load_crop(crop_file)

    # ── Field shape: south edge fixed, north edge sheared sideways ───────────
    # rotation_deg tilts only the north end of each row; (ref_x, y_south) and
    # the west edge at y_south never move, so the field is a parallelogram,
    # not a rotated rectangle.
    stripe_total_w = MAX_GREEN_STRIPES * STRIPE_GREEN_W + (MAX_GREEN_STRIPES - 1) * STRIPE_WHITE_W
    dx_shift = 0.0

    if crop:
        theta    = math.radians(crop.get('rotation_deg', 0.0))
        dx_shift = (crop['y_north'] - crop['y_south']) * math.sin(theta)
        west_x   = crop['ref_x'] - stripe_total_w

        corners = [
            (crop['ref_x'], crop['y_south']), (west_x,            crop['y_south']),
            (crop['ref_x'] + dx_shift, crop['y_north']), (west_x + dx_shift, crop['y_north']),
        ]
        xs, ys = zip(*corners)
        x_min, x_max = min(xs) - CROP_MARGIN, max(xs) + CROP_MARGIN
        y_min, y_max = min(ys) - CROP_MARGIN, max(ys) + CROP_MARGIN

        # The field shape is only the *default* tight fit — expand to include
        # all current/past trajectory data so previous visits are never clipped.
        def _expand(wx, wy):
            nonlocal x_min, x_max, y_min, y_max
            x_min = min(x_min, wx - CROP_MARGIN); x_max = max(x_max, wx + CROP_MARGIN)
            y_min = min(y_min, wy - CROP_MARGIN); y_max = max(y_max, wy + CROP_MARGIN)

        for p in track:
            _expand(p['x'], p['y'])
        if past_paths:
            for path in past_paths:
                for p in path:
                    _expand(p['x'], p['y'])
        if landmarks:
            for lm in landmarks:
                if lm.get('lat') is None:
                    continue
                lx, ly = _gps_to_enu(lm['lat'], lm['lon'], origin_lat, origin_lon)
                _expand(lx, ly)
        if waypoints:
            for wp in waypoints:
                _expand(wp.get('x', 0), wp.get('y', 0))
    else:
        # ── Fallback (no crop configured yet): legacy North-up bounds ────────
        PAD = 20.0
        x_min, y_min, x_max, y_max = localizer.get_map_bounds_enu(padding_m=PAD)

        if past_paths:
            for path in past_paths:
                for p in path:
                    x_min = min(x_min, p['x'] - PAD)
                    y_min = min(y_min, p['y'] - PAD)
                    x_max = max(x_max, p['x'] + PAD)
                    y_max = max(y_max, p['y'] + PAD)

        if landmarks:
            for lm in landmarks:
                if lm.get('lat') is None:
                    continue
                lx, ly = _gps_to_enu(lm['lat'], lm['lon'], origin_lat, origin_lon)
                x_min = min(x_min, lx - PAD)
                y_min = min(y_min, ly - PAD)
                x_max = max(x_max, lx + PAD)
                y_max = max(y_max, ly + PAD)

        # Ensure minimum 60 × 60 m
        cx  = (x_min + x_max) / 2
        cy  = (y_min + y_max) / 2
        ext = max(30.0, (x_max - x_min) / 2, (y_max - y_min) / 2)
        x_min, x_max = cx - ext, cx + ext
        y_min, y_max = cy - ext, cy + ext

    W = min(MAX_IMG_PX, max(200, int((x_max - x_min) / RESOLUTION)))
    H = min(MAX_IMG_PX, max(200, int((y_max - y_min) / RESOLUTION)))

    def to_px(x, y):
        px = int((x - x_min) / RESOLUTION)
        py = H - 1 - int((y - y_min) / RESOLUTION)
        return px, py

    img  = Image.new('RGB', (W, H), C_BG)
    draw = ImageDraw.Draw(img)

    # Field stripes from crop_field.json (permanent — never from mission points).
    # South edge is fixed; north edge is shifted by dx_shift → each stripe is
    # drawn as a parallelogram (south corners never move).
    # Left of ref_x: green 9ft / white 5ft alternating.
    if crop:
        y_s, y_n  = crop['y_south'], crop['y_north']
        cur_lx    = 0.0
        for _ in range(MAX_GREEN_STRIPES):
            g_right_lx = cur_lx
            g_left_lx  = cur_lx - STRIPE_GREEN_W
            quad = [
                to_px(crop['ref_x'] + g_right_lx,            y_s),
                to_px(crop['ref_x'] + g_left_lx,              y_s),
                to_px(crop['ref_x'] + g_left_lx  + dx_shift,  y_n),
                to_px(crop['ref_x'] + g_right_lx + dx_shift,  y_n),
            ]
            draw.polygon(quad, fill=C_FIELD_GREEN)
            cur_lx = g_left_lx - STRIPE_WHITE_W

    # Grid
    x0 = math.ceil(x_min / GRID_SPACING) * GRID_SPACING
    y0 = math.ceil(y_min / GRID_SPACING) * GRID_SPACING
    for gx in np.arange(x0, x_max + GRID_SPACING, GRID_SPACING):
        px, _ = to_px(gx, y_min)
        draw.line([(px, 0), (px, H - 1)], fill=C_GRID, width=1)
    for gy in np.arange(y0, y_max + GRID_SPACING, GRID_SPACING):
        _, py = to_px(x_min, gy)
        draw.line([(0, py), (W - 1, py)], fill=C_GRID, width=1)

    # N arrow (top-right corner, always points straight up — North-up frame)
    ax, ay = W - 30, 30
    draw.line([(ax, ay + 15), (ax, ay - 15)], fill=(100, 100, 200), width=2)
    draw.polygon([(ax - 4, ay - 10), (ax + 4, ay - 10), (ax, ay - 20)], fill=(100, 100, 200))
    draw.text((ax - 3, ay - 32), 'N', fill=(60, 60, 160))

    # Past paths: each path 10% opacity, overlapping areas accumulate (heavily-used routes darken)
    C_PAST = (80, 50, 10)
    if past_paths:
        # Rasterize each path into a binary coverage layer, accumulate pixel counts
        coverage = np.zeros((H, W), dtype=np.float32)
        for path in past_paths:
            if len(path) < 2:
                continue
            layer = Image.new('L', (W, H), 0)
            ldraw = ImageDraw.Draw(layer)
            step  = max(1, len(path) // 2000)
            pts   = [to_px(p['x'], p['y']) for p in path[::step]]
            for i in range(1, len(pts)):
                ldraw.line([pts[i - 1], pts[i]], fill=255, width=2)
            coverage += (np.array(layer, dtype=np.float32) > 0)

        # alpha = 1 − 0.9^N  →  N overlapping paths compound to 10% opacity each
        alpha   = 1.0 - np.power(0.9, coverage)          # shape (H, W)
        img_arr = np.array(img, dtype=np.float32)
        pc      = np.array(C_PAST, dtype=np.float32)
        img_arr = img_arr * (1.0 - alpha[:, :, None]) + pc[None, None, :] * alpha[:, :, None]
        img  = Image.fromarray(np.clip(img_arr, 0, 255).astype(np.uint8))
        draw = ImageDraw.Draw(img)

    # GPS trail (current session, full opacity)
    if len(track) >= 2:
        step = max(1, len(track) // 3000)
        pts  = [to_px(p['x'], p['y']) for p in track[::step]]
        for i in range(1, len(pts)):
            draw.line([pts[i - 1], pts[i]], fill=C_TRAIL, width=2)

    # Waypoints
    if waypoints:
        wp_pts = []
        for wp in waypoints:
            wx, wy = wp.get('x', 0), wp.get('y', 0)
            wp_pts.append(to_px(wx, wy))
        if len(wp_pts) >= 2:
            for i in range(1, len(wp_pts)):
                draw.line([wp_pts[i - 1], wp_pts[i]],
                          fill=(*C_WAYPOINT, 180), width=1)
        for i, p in enumerate(wp_pts):
            r = 3 if i == 0 else 2
            draw.ellipse([p[0]-r, p[1]-r, p[0]+r, p[1]+r],
                         fill=C_WAYPOINT, outline=(255, 255, 255))

    # Landmarks — TEMP (user-requested): not drawn while landmark accuracy work
    # continues. Re-enable by removing this `if not LANDMARKS_PAUSED:` guard.
    if not LANDMARKS_PAUSED:
        for lm in landmarks:
            if lm.get('lat') is None:
                continue
            lx, ly = _gps_to_enu(lm['lat'], lm['lon'], origin_lat, origin_lon)
            px, py = to_px(lx, ly)
            colour = TYPE_COLOURS.get(lm['type'], C_DEFAULT_LM)

            if lm['type'] == 'crop_stick':
                r = 4
                draw.ellipse([px-r, py-r, px+r, py+r], fill=colour, outline=(0, 0, 0))
                num = lm.get('number')
                if num:
                    draw.text((px + 5, py - 6), str(num), fill=(0, 0, 0))
            else:
                # Draw bbox if available
                if all(k in lm for k in ('lat_min', 'lat_max', 'lon_min', 'lon_max')):
                    x0l, y0l = _gps_to_enu(lm['lat_min'], lm['lon_min'], origin_lat, origin_lon)
                    x1l, y1l = _gps_to_enu(lm['lat_max'], lm['lon_max'], origin_lat, origin_lon)
                    p0 = to_px(x0l, y0l)
                    p1 = to_px(x1l, y1l)
                    x_left   = min(p0[0], p1[0])
                    x_right  = max(p0[0], p1[0])
                    y_top    = min(p0[1], p1[1])
                    y_bottom = max(p0[1], p1[1])
                    draw.rectangle([x_left, y_top, x_right, y_bottom],
                                   outline=colour, width=2)
                else:
                    r = 8
                    draw.rectangle([px-r, py-r, px+r, py+r], outline=colour, width=2)
                label = lm.get('description') or lm['type']
                draw.text((px + 10, py - 6), label[:20], fill=colour)

    # Robot — circle sized to the 0.6 x 0.6 m footprint, heading shown as a line
    if robot_pose:
        rwx = robot_pose.get('x', 0)
        rwy = robot_pose.get('y', 0)
        yaw = robot_pose.get('yaw', 0)
        heading_valid = robot_pose.get('heading_valid', True)
        rpx, rpy = to_px(rwx, rwy)

        rr = ROBOT_HALF_M / RESOLUTION
        draw.ellipse([rpx - rr, rpy - rr, rpx + rr, rpy + rr],
                     fill=C_ROBOT, outline=(255, 255, 255))

        if heading_valid:
            alen = rr + 0.15 / RESOLUTION
            dx =  alen * math.cos(yaw)
            dy = -alen * math.sin(yaw)
            draw.line([(rpx, rpy), (rpx + dx, rpy + dy)], fill=(255, 255, 255), width=2)

    # Save
    os.makedirs(output_dir, exist_ok=True)
    png_path   = os.path.join(output_dir, 'map_latest.png')
    state_path = os.path.join(output_dir, 'map_state.json')
    img.save(png_path, 'PNG')

    map_state = {
        'resolution': RESOLUTION,
        'origin_x':   float(x_min),
        'origin_y':   float(y_min),
        'img_width':  W,
        'img_height': H,
        'rot_angle':  0.0,
    }
    with open(state_path, 'w') as f:
        json.dump(map_state, f)

    return map_state


class GpsMapLoop:
    """
    Background thread that periodically regenerates the GPS-based map
    and sends it to the internet comm (which pushes it to the web panel).
    """

    def __init__(self, localizer, store, internet, output_dir: str,
                 interval: float = MAP_INTERVAL, crop_file: str | None = None):
        self._localizer  = localizer
        self._store      = store
        self._internet   = internet
        self._output_dir = output_dir
        self._interval   = interval
        self._crop_file  = crop_file   # path to crop_field.json (permanent, never changes)
        self._robot_pose: dict | None = None
        self._waypoints:  list | None = None

    def update_robot_pose(self, pose: dict):
        self._robot_pose = pose

    def update_waypoints(self, waypoints: list):
        self._waypoints = waypoints

    def start(self):
        threading.Thread(target=self._loop, daemon=True, name='gps-map').start()
        log.info('GpsMapLoop started')

    def _loop(self):
        while True:
            try:
                state = generate_map(
                    self._localizer,
                    self._store,
                    self._output_dir,
                    robot_pose=self._robot_pose,
                    waypoints=self._waypoints,
                    past_paths=self._localizer.get_past_paths(),
                    crop_file=self._crop_file,
                )
                if state and self._internet.connected:
                    png_path = os.path.join(self._output_dir, 'map_latest.png')
                    self._internet.send_map(png_path, state)
            except Exception as e:
                log.warning(f'GpsMapLoop: {e}')
            time.sleep(self._interval)
