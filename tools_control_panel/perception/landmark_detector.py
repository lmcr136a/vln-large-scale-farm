"""
LandmarkDetector: YOLO-World based landmark detection from RGB cameras + LiDAR depth.

Backend (set in farm_config.yaml):
  yolo   — YOLO-World real-time detection (default, recommended)
  ollama — Qwen VLM via Ollama (slow, fallback)
  claude — Anthropic API (needs internet)

YOLO-World setup:
  pip install ultralytics
  # weights auto-downloaded on first run (~50 MB for -s variant)

Example config:
  yolo:
    model:         'yolov8s-worldv2.pt'   # -s fast, -m more accurate
    classes:
      - 'metal storage box'
      - 'trailer'
      - 'marker stake'
      - 'sign post'
      - 'barrel'
    conf_threshold: 0.25
    interval_sec:   2.0
    input_size:     640
"""
import base64
import logging
import math
import threading
import time

import cv2
import numpy as np

log = logging.getLogger(__name__)

# ── Constants ─────────────────────────────────────────────────────────────────

# Rotation from ZED camera frame → robot body frame.
# ZED: X=right, Y=down, Z=forward (optical axis).
# Robot: x=forward, y=left, z=up.
# Mounting: front/back rotate 180° around vertical; left/right rotate ±90°.
_CAM_TO_ROBOT = {
    'front': np.array([[ 0,  0,  1],   # robot_x = cam_z  (optical axis → forward)
                       [-1,  0,  0],   # robot_y = -cam_x (image-right → robot-right = -left)
                       [ 0, -1,  0]], dtype=np.float64),  # robot_z = -cam_y
    'back':  np.array([[ 0,  0, -1],   # robot_x = -cam_z
                       [ 1,  0,  0],   # robot_y =  cam_x
                       [ 0, -1,  0]], dtype=np.float64),
    'left':  np.array([[ 1,  0,  0],   # robot_x =  cam_x (image-right → robot-forward)
                       [ 0,  0,  1],   # robot_y =  cam_z (optical axis → robot-left)
                       [ 0, -1,  0]], dtype=np.float64),
    'right': np.array([[-1,  0,  0],   # robot_x = -cam_x
                       [ 0,  0, -1],   # robot_y = -cam_z
                       [ 0, -1,  0]], dtype=np.float64),
}

# Base yaw offset per camera (added to robot heading → world direction)
# ENU convention: 0=East, π/2=North, counterclockwise positive
CAMERA_YAW_OFFSET = {
    'front':  0.0,
    'left':   math.pi / 2,
    'back':   math.pi,
    'right': -math.pi / 2,
}

# Sign of horizontal bbox offset → ENU yaw change
# front/left cameras: image-right = robot-right = clockwise = negative yaw
# back/right cameras: image-right = robot-left  = counterclockwise = positive yaw
CAM_H_SIGN = {'front': -1, 'back': +1, 'left': -1, 'right': +1}

# Default camera translations from LiDAR (robot frame: x=fwd, y=left, z=up)
_DEFAULT_CAM_T = {
    'front': [ 0.10,  0.00, -0.08],
    'back':  [-0.10,  0.00, -0.08],
    'left':  [ 0.00,  0.15, -0.15],
    'right': [ 0.00, -0.15, -0.15],
}

# Camera look-direction axes for LiDAR FOV filtering
# (depth_axis, depth_sign, lateral_axis)
_CAM_AXES = {
    'front': (0,  1, 1),
    'back':  (0, -1, 1),
    'left':  (1,  1, 0),
    'right': (1, -1, 0),
}

# Default YOLO-World classes (farm-relevant structures only, no people/animals)
DEFAULT_YOLO_CLASSES = [
    'metal storage box',
    'metal container',
    'trailer',
    'large farm equipment',
    'marker stake',
    'numbered pole',
    'sign post',
    'barrel',
    'pallet',
]

# ── Triangulation helpers ─────────────────────────────────────────────────────

def _ray_intersect(rays: list) -> tuple | None:
    """Least-squares N-ray intersection in 2D ENU plane.

    Each ray is (origin_x, origin_y, angle_rad).
    Minimises sum of squared perpendicular distances to all rays.
    Returns (x, y) or None if the system is degenerate (parallel rays).
    """
    A = np.zeros((2, 2), dtype=np.float64)
    b = np.zeros(2,      dtype=np.float64)
    for rx, ry, ang in rays:
        d = np.array([math.cos(ang), math.sin(ang)], dtype=np.float64)
        M = np.eye(2) - np.outer(d, d)     # (I − d dᵀ): projects onto perpendicular
        p = np.array([rx, ry], dtype=np.float64)
        A += M
        b += M @ p
    try:
        xy = np.linalg.solve(A, b)
        return float(xy[0]), float(xy[1])
    except np.linalg.LinAlgError:
        return None


class _RayBuffer:
    """Accumulates per-observation rays and triangulates landmark positions.

    Each detection is a ray: (robot_enu_x, robot_enu_y, world_direction_rad).
    Rays pointing to the same landmark are clustered by their rough intersection.
    A position is emitted only after MIN_RAYS observations from sufficiently
    spread robot positions — eliminating single-frame distance/heading noise.
    """

    CLUSTER_R = 30.0   # m: rays within this radius share a cluster
    MAX_AGE   = 120.0  # s: drop observations older than this
    MIN_MOVE  = 2.0    # m: minimum robot displacement between observations
    MIN_RAYS  = 3      # observations required before emitting a position

    def __init__(self):
        self._clusters: dict[int, dict] = {}
        self._next_id = 0

    def add(self, cls: str, robot_x: float, robot_y: float,
            angle_rad: float, hint_dist: float = 15.0) -> tuple | None:
        """Register an observation ray.

        Returns (enu_x, enu_y, n_rays) when triangulation is ready, else None.
        hint_dist is used only for initial cluster matching, not the final position.
        """
        now   = time.time()
        est_x = robot_x + hint_dist * math.cos(angle_rad)
        est_y = robot_y + hint_dist * math.sin(angle_rad)

        # Prune stale rays from all clusters
        for cid in list(self._clusters):
            c = self._clusters[cid]
            c['rays'] = [(t, x, y, a) for t, x, y, a in c['rays']
                         if now - t < self.MAX_AGE]
            if not c['rays']:
                del self._clusters[cid]

        # Find nearest cluster of the same class
        best_id, best_d = None, float('inf')
        for cid, c in self._clusters.items():
            if c['cls'] != cls:
                continue
            ex, ey = c['est']
            d = math.hypot(est_x - ex, est_y - ey)
            if d < self.CLUSTER_R and d < best_d:
                best_id, best_d = cid, d

        if best_id is None:
            best_id = self._next_id
            self._next_id += 1
            self._clusters[best_id] = {'cls': cls, 'rays': [], 'est': (est_x, est_y)}

        rays = self._clusters[best_id]['rays']

        # Skip if robot hasn't moved enough since the last observation
        if rays:
            _, lx, ly, _ = rays[-1]
            if math.hypot(robot_x - lx, robot_y - ly) < self.MIN_MOVE:
                return None

        rays.append((now, robot_x, robot_y, angle_rad))

        if len(rays) < self.MIN_RAYS:
            log.debug(f'RayBuffer [{cls}] cluster {best_id}: '
                      f'{len(rays)}/{self.MIN_RAYS} rays accumulated')
            return None

        xy = _ray_intersect([(rx, ry, a) for _, rx, ry, a in rays])
        if xy is None:
            return None

        # Sanity check: result must be in front of at least one observer
        _, ox, oy, oa = rays[-1]
        if math.hypot(xy[0] - ox, xy[1] - oy) > 200.0:
            log.debug(f'RayBuffer [{cls}]: triangulation result too far, discarding')
            return None

        self._clusters[best_id]['est'] = xy
        return (*xy, len(rays))


# Map detected class name → landmark type
def _to_lm_type(cls_name: str) -> str:
    n = cls_name.lower()
    if any(k in n for k in ('trailer', 'equipment', 'vehicle', 'truck')):
        return 'trailer'
    if any(k in n for k in ('stake', 'pole', 'sign', 'marker', 'number', 'post')):
        return 'crop_stick'
    return 'metal_box'


# ── Detector ──────────────────────────────────────────────────────────────────

class LandmarkDetector:
    def __init__(self, localizer, store, config: dict):
        self._localizer = localizer
        self._store     = store

        # ── Backend selection ────────────────────────────────────────────────
        yolo_cfg = config.get('yolo', {})
        vlm_cfg  = config.get('vlm',  {})

        # yolo section in config → use YOLO; otherwise fall back to vlm.backend
        if yolo_cfg.get('model') or yolo_cfg.get('enabled', True):
            self._backend = 'yolo'
        else:
            self._backend = vlm_cfg.get('backend', 'ollama')

        # ── YOLO config ──────────────────────────────────────────────────────
        self._yolo_model_name  = yolo_cfg.get('model', 'yolov8s-worldv2.pt')
        self._yolo_classes     = yolo_cfg.get('classes', DEFAULT_YOLO_CLASSES)
        self._conf_threshold   = float(yolo_cfg.get('conf_threshold', 0.25))
        self._interval         = float(yolo_cfg.get('interval_sec',   2.0))
        self._input_size       = int(yolo_cfg.get('input_size',       640))
        self._yolo_model       = None   # lazy-loaded

        # ── VLM fallback config ──────────────────────────────────────────────
        self._vlm_backend    = vlm_cfg.get('backend',     'ollama')
        self._model          = vlm_cfg.get('model',       'qwen2.5vl:3b')
        self._api_key        = vlm_cfg.get('api_key',     '')
        self._ollama_url     = vlm_cfg.get('ollama_url',  'http://localhost:11434')
        self._keep_alive     = vlm_cfg.get('keep_alive',  '10m')
        self._vlm_timeout    = float(vlm_cfg.get('timeout_sec', 150.0))

        # ── Enabled flag ─────────────────────────────────────────────────────
        if self._backend == 'yolo':
            self._enabled = True
        elif self._backend == 'claude':
            self._enabled = bool(self._api_key)
        elif self._backend == 'ollama':
            self._enabled = True
        else:
            self._enabled = False

        # If vlm section has its own interval and yolo section is absent, use it
        if self._backend != 'yolo':
            self._interval = float(vlm_cfg.get('interval_sec', 120.0))

        # ── Camera extrinsics ────────────────────────────────────────────────
        cx = config.get('camera_extrinsics', {})
        self._cam_t = {
            cam: np.array(cx.get(cam, _DEFAULT_CAM_T[cam]), dtype=np.float32)
            for cam in ('front', 'back', 'left', 'right')
        }
        self._cam_hfov = math.radians(float(cx.get('hfov_deg', 55.0)))

        # ── State ────────────────────────────────────────────────────────────
        self._lock       = threading.Lock()
        self._images: dict[str, tuple[float, bytes]] = {}   # cam → (timestamp, jpeg)
        # cam → (timestamp, float32 depth array in metres, intrinsics dict {fx,fy,cx,cy})
        self._depths: dict[str, tuple[float, np.ndarray, dict]] = {}
        self._pointcloud: np.ndarray | None = None
        self._running    = False
        self._ray_buffer = _RayBuffer()
        # Optional callback(camera: str, jpeg_bytes: bytes) for annotated frames
        self._on_annotated_frame = None

    # ── Feed data ─────────────────────────────────────────────────────────────

    def on_image(self, camera: str, jpeg_bytes: bytes):
        with self._lock:
            self._images[camera] = (time.time(), jpeg_bytes)

    def on_image_b64(self, camera: str, b64: str):
        try:
            self.on_image(camera, base64.b64decode(b64))
        except Exception:
            pass

    def on_depth_frame(self, camera: str, depth_arr: np.ndarray, intrinsics: dict):
        """Store a ZED depth map (float32, metres, NaN=invalid) for landmark localization."""
        with self._lock:
            self._depths[camera] = (time.time(), depth_arr, intrinsics)

    def on_pointcloud(self, points: np.ndarray):
        with self._lock:
            self._pointcloud = points

    # ── Control ───────────────────────────────────────────────────────────────

    def start(self):
        if not self._enabled:
            log.info('LandmarkDetector disabled (check config)')
            return
        if self._backend == 'yolo':
            if not self._load_yolo():
                self._enabled = False
                return
        elif self._backend == 'ollama' and not self._check_ollama():
            log.warning('LandmarkDetector: Ollama not reachable — disabled')
            self._enabled = False
            return
        self._running = True
        threading.Thread(target=self._loop, daemon=True,
                         name='landmark-detector').start()
        log.info(f'LandmarkDetector started [{self._backend}] '
                 f'interval={self._interval}s classes={self._yolo_classes if self._backend=="yolo" else self._model}')

    def stop(self):
        self._running = False

    def is_enabled(self) -> bool:
        return self._enabled

    def set_annotated_frame_callback(self, cb):
        """cb(camera: str, jpeg_bytes: bytes) — called after each YOLO inference."""
        self._on_annotated_frame = cb

    # ── Internal loop ─────────────────────────────────────────────────────────

    def _loop(self):
        while self._running:
            time.sleep(self._interval)
            if not self._images_fresh():
                log.debug(f'LandmarkDetector: no fresh images '
                          f'(cameras seen: {list(self._images.keys())})')
                continue
            try:
                self._detect_once()
            except Exception as e:
                log.warning(f'LandmarkDetector: {e}')

    def _images_fresh(self, max_age: float = 10.0) -> bool:
        now = time.time()
        with self._lock:
            if not self._images:
                return False
            return all(now - ts < max_age for ts, _ in self._images.values())

    def _detect_once(self):
        with self._lock:
            snapshot = {cam: (ts, bytes(data))
                        for cam, (ts, data) in self._images.items()}

        if self._backend == 'yolo':
            # YOLO always runs (sends annotated frames regardless of GPS)
            detections = self._run_yolo(snapshot)
            if not detections:
                return
            # Landmark saving requires valid GPS heading (unreliable until robot moves)
            if not self._localizer.is_heading_valid():
                log.debug('LandmarkDetector: heading not yet valid, skipping save')
                return
            gps = self._localizer.get_current_gps()
            if gps is None:
                return
            lat, lon, heading = gps
        else:
            gps = self._localizer.get_current_gps()
            if gps is None:
                log.warning('LandmarkDetector: GPS not available, skipping')
                return
            lat, lon, heading = gps
            images_b64 = {cam: base64.b64encode(data).decode()
                          for cam, (_, data) in snapshot.items()}
            raw = self._call_vlm(images_b64)
            if not raw:
                return
            detections = self._vlm_to_detections(raw, {})

        # Get robot ENU position for ray registration
        enu = self._localizer.get_current_enu()
        if enu is None:
            return
        robot_x, robot_y, _ = enu

        changed = False
        for det in detections:
            cam     = det['camera']
            h_angle = det.get('h_angle', 0.0)
            bbox    = det.get('bbox_xyxy')

            # ── Try ZED depth-based single-frame positioning (most accurate) ──
            depth_result = None
            if bbox and self._depths.get(cam) is not None:
                depth_result = self._position_from_depth(
                    cam, *bbox, robot_x, robot_y, heading
                )

            if depth_result is not None:
                obj_x, obj_y, Z, fx = depth_result
                obj_lat, obj_lon = self._localizer.enu_to_gps(obj_x, obj_y)
                if obj_lat is None:
                    continue

                # Width from pixel span and actual depth: w = (x2-x1) * Z / fx
                width_m = max(0.3, min(30.0, (bbox[2] - bbox[0]) * Z / fx))

                lm_id = self._store.add_landmark(
                    lm_type     = det['type'],
                    lat         = obj_lat,
                    lon         = obj_lon,
                    description = det.get('description', ''),
                    number      = det.get('number'),
                    width_m     = width_m,
                    height_m    = width_m,
                    confidence  = det.get('conf', 0.7),
                )
                log.info(f'Landmark {lm_id} [{det["type"]}] depth={Z:.2f}m'
                         f' → ({obj_lat:.6f}, {obj_lon:.6f}) width={width_m:.2f}m')
                changed = True

            else:
                # ── Fallback: ray triangulation from multiple robot positions ──
                world_dir = heading + CAMERA_YAW_OFFSET.get(cam, 0.0) + h_angle
                hint      = self._lidar_distance_for_detection(cam, h_angle) or 15.0
                result    = self._ray_buffer.add(
                    det['description'], robot_x, robot_y, world_dir, hint_dist=hint
                )
                if result is None:
                    continue

                obj_x, obj_y, n_rays = result
                obj_lat, obj_lon = self._localizer.enu_to_gps(obj_x, obj_y)
                if obj_lat is None:
                    continue

                tri_dist    = math.hypot(obj_x - robot_x, obj_y - robot_y)
                bbox_w_frac = det.get('bbox_w_frac', 0.15)
                half_angle  = bbox_w_frac * self._cam_hfov
                width_m     = max(0.5, min(30.0, 2 * tri_dist * math.tan(half_angle)))

                lm_id = self._store.add_landmark(
                    lm_type     = det['type'],
                    lat         = obj_lat,
                    lon         = obj_lon,
                    description = det.get('description', ''),
                    number      = det.get('number'),
                    width_m     = width_m,
                    height_m    = width_m,
                    confidence  = det.get('conf', 0.7),
                )
                log.info(f'Landmark {lm_id} [{det["type"]}] triangulated from {n_rays} rays'
                         f' → ({obj_lat:.6f}, {obj_lon:.6f})')
                changed = True

        if changed:
            self._store.save()

    # ── YOLO-World backend ────────────────────────────────────────────────────

    def _load_yolo(self) -> bool:
        try:
            from ultralytics import YOLOWorld
            self._yolo_model = YOLOWorld(self._yolo_model_name)
            self._yolo_model.set_classes(self._yolo_classes)
            # Warm-up pass
            dummy = np.zeros((self._input_size, self._input_size, 3), dtype=np.uint8)
            self._yolo_model(dummy, verbose=False)
            log.info(f'YOLO-World loaded: {self._yolo_model_name} '
                     f'classes={self._yolo_classes}')
            return True
        except ImportError:
            log.error('ultralytics not installed: pip install ultralytics')
            return False
        except Exception as e:
            log.error(f'YOLO-World load failed: {e}')
            return False

    def _run_yolo(self, snapshot: dict) -> list[dict]:
        """Run YOLO-World on each camera image. Returns list of detection dicts."""
        detections = []
        for cam, (_, jpeg) in snapshot.items():
            buf = np.frombuffer(jpeg, np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if img is None:
                continue
            results = self._yolo_model(
                img,
                imgsz   = self._input_size,
                conf    = self._conf_threshold,
                verbose = True,
            )
            log.info(f'YOLO [{cam}]: {len(results[0].boxes)} detections')

            # Annotate frame (seg mask if available, bbox otherwise) and stream back
            if self._on_annotated_frame is not None:
                try:
                    ann = results[0].plot()          # BGR numpy, auto seg/bbox
                    ok, enc = cv2.imencode(
                        '.jpg', ann, [cv2.IMWRITE_JPEG_QUALITY, 85]
                    )
                    if ok:
                        self._on_annotated_frame(cam, enc.tobytes())
                except Exception as e:
                    log.debug(f'annotated frame encode failed [{cam}]: {e}')

            img_w = img.shape[1]
            for box in results[0].boxes:
                cls_idx  = int(box.cls[0])
                cls_name = self._yolo_classes[cls_idx] \
                           if cls_idx < len(self._yolo_classes) \
                           else results[0].names.get(cls_idx, 'unknown')
                conf     = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                # Horizontal angle offset from bbox centre
                bbox_cx = (x1 + x2) / 2.0
                norm_x  = (bbox_cx - img_w / 2.0) / (img_w / 2.0)
                h_angle = CAM_H_SIGN.get(cam, -1) * norm_x * self._cam_hfov

                # Bbox width as fraction of image — used later for real-world size
                bbox_w_frac = (x2 - x1) / img_w

                detections.append({
                    'camera':       cam,
                    'type':         _to_lm_type(cls_name),
                    'description':  cls_name,
                    'conf':         conf,
                    'h_angle':      h_angle,
                    'bbox_w_frac':  bbox_w_frac,
                    'bbox_xyxy':    (int(x1), int(y1), int(x2), int(y2)),
                })
        return detections

    # ── Depth-based 3D localization ───────────────────────────────────────────

    def _position_from_depth(self, cam: str,
                              x1: float, y1: float, x2: float, y2: float,
                              robot_x: float, robot_y: float,
                              heading: float) -> tuple | None:
        """Back-project a detected bbox to ENU world coordinates using ZED depth.

        Returns (enu_x, enu_y, depth_m, fx) or None when depth is unavailable/invalid.

        Pipeline:
          pixel (u,v) + depth Z → camera 3D point
            → robot body frame (via _CAM_TO_ROBOT rotation + camera translation)
            → ENU frame (rotate by robot heading)
            → absolute ENU (add robot position)
        """
        with self._lock:
            entry = self._depths.get(cam)
        if entry is None:
            return None
        _, depth_arr, K = entry

        # Bbox centre pixel (clamp to image bounds)
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)
        h_arr, w_arr = depth_arr.shape[:2]
        u = max(0, min(u, w_arr - 1))
        v = max(0, min(v, h_arr - 1))

        # Sample median depth in a ±12 px patch for robustness against single bad pixels
        r = 12
        patch = depth_arr[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
        valid = patch[np.isfinite(patch) & (patch > 0.3) & (patch < 80.0)]
        if len(valid) < 10:
            return None
        Z = float(np.median(valid))

        # Pinhole back-projection in camera frame (ZED: X=right, Y=down, Z=forward)
        X_cam = (u - K['cx']) * Z / K['fx']
        Y_cam = (v - K['cy']) * Z / K['fy']
        p_cam = np.array([X_cam, Y_cam, Z], dtype=np.float64)

        # Camera frame → robot body frame
        R = _CAM_TO_ROBOT.get(cam)
        if R is None:
            return None
        p_body = R @ p_cam + self._cam_t[cam].astype(np.float64)

        # Robot body frame → ENU (rotate by heading; body x=fwd→heading, y=left)
        # ENU_dx = body_x·cos(h) - body_y·sin(h)
        # ENU_dy = body_x·sin(h) + body_y·cos(h)
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        enu_dx = p_body[0] * cos_h - p_body[1] * sin_h
        enu_dy = p_body[0] * sin_h + p_body[1] * cos_h

        return robot_x + enu_dx, robot_y + enu_dy, Z, K['fx']

    # ── GPS projection ────────────────────────────────────────────────────────

    def _project(self, lat, lon, heading_rad, camera, dist_m,
                 h_angle: float = 0.0):
        """Project detected object to GPS coordinates.

        h_angle: horizontal angle offset in radians from camera centre
                 (positive = counterclockwise = left in ENU)
        """
        direction = heading_rad + CAMERA_YAW_OFFSET.get(camera, 0.0) + h_angle
        dlat = dist_m * math.sin(direction) / 6378137.0
        dlon = dist_m * math.cos(direction) / (6378137.0 * math.cos(math.radians(lat)))
        return lat + math.degrees(dlat), lon + math.degrees(dlon)

    # ── LiDAR depth ───────────────────────────────────────────────────────────

    def _lidar_distance_for_detection(self, cam: str, h_angle: float,
                                      cone_deg: float = 15.0) -> float | None:
        """LiDAR distance in the exact angular direction of a detected bbox.

        Uses robot-frame direction = CAMERA_YAW_OFFSET[cam] + h_angle,
        filters to a ±cone_deg cone around that direction, and returns the
        30th-percentile distance (avoids both ground hits and far-field noise).
        Ground points (z < 0.3 m) are excluded before querying.
        """
        with self._lock:
            pts = self._pointcloud
        if pts is None or len(pts) < 20:
            return None

        # Remove ground points — LiDAR height filter
        above = pts[pts[:, 2] > 0.3]
        if len(above) < 5:
            return None

        xyz = above[:, :3]
        t   = self._cam_t[cam]
        p   = xyz - t                         # translate to camera origin

        # Robot-frame direction of this detection (0=fwd, π/2=left)
        robot_dir = CAMERA_YAW_OFFSET[cam] + h_angle
        dir_x = math.cos(robot_dir)
        dir_y = math.sin(robot_dir)

        # Signed angle of each LiDAR point from robot-forward in XY plane
        point_dir  = np.arctan2(p[:, 1], p[:, 0])
        angle_diff = np.arctan2(np.sin(point_dir - robot_dir),
                                np.cos(point_dir - robot_dir))

        # Forward distance along detection axis
        forward = p[:, 0] * dir_x + p[:, 1] * dir_y

        cone_rad = math.radians(cone_deg)
        mask = (forward > 0.5) & (np.abs(angle_diff) < cone_rad)

        # Widen cone if too sparse
        if mask.sum() < 3:
            mask = (forward > 0.5) & (np.abs(angle_diff) < cone_rad * 2)
        if mask.sum() < 3:
            return None

        dists = np.linalg.norm(p[mask], axis=1)
        return float(np.percentile(dists, 30))

    # ── VLM backends (fallback) ───────────────────────────────────────────────

    def _vlm_to_detections(self, raw: dict, lidar_dists: dict) -> list[dict]:
        """Convert VLM JSON output to unified detection list."""
        out = []
        for lm in raw.get('landmarks', []):
            cam  = lm.get('camera', 'front')
            dist = lidar_dists.get(cam) or float(lm.get('distance_m', 10.0))
            out.append({'camera': cam, 'type': lm.get('type', 'metal_box'),
                        'description': lm.get('description', ''),
                        'distance_m': dist, 'conf': 0.6})
        for st in raw.get('sticks', []):
            cam  = st.get('camera', 'front')
            dist = lidar_dists.get(cam) or float(st.get('distance_m', 3.0))
            num  = st.get('number')
            out.append({'camera': cam, 'type': 'crop_stick',
                        'description': f'Stick #{num}' if num else 'Crop stick',
                        'number': str(num) if num else None,
                        'distance_m': dist, 'conf': 0.6})
        return out

    def _check_ollama(self) -> bool:
        try:
            import requests
            r = requests.get(f'{self._ollama_url}/api/tags', timeout=3)
            return r.status_code == 200
        except Exception as e:
            log.warning(f'Ollama check failed: {e}')
            return False

    def _call_vlm(self, images_b64: dict) -> dict | None:
        if self._vlm_backend == 'ollama':
            return self._call_ollama(images_b64)
        if self._vlm_backend == 'claude':
            return self._call_claude(images_b64)
        return None

    def _call_ollama(self, images_b64: dict) -> dict | None:
        try:
            import requests
        except ImportError:
            return None
        ordered = [self._resize_jpeg(images_b64[c])
                   for c in ('front', 'back', 'left', 'right') if c in images_b64]
        if not ordered:
            return None
        PROMPT = ('Farm robot cameras (1=FRONT 2=BACK 3=LEFT 4=RIGHT). '
                  'JSON only: {"landmarks":[{"camera":"front","type":"metal_box","distance_m":8}],'
                  '"sticks":[{"camera":"left","number":"42","distance_m":3}]} '
                  'Types: metal_box|trailer|vehicle. Skip people.')
        payload = {
            'model': self._model, 'prompt': PROMPT,
            'images': ordered, 'stream': False, 'format': 'json',
            'keep_alive': self._keep_alive,
            'options': {'temperature': 0.05, 'num_predict': 256},
        }
        try:
            resp = requests.post(f'{self._ollama_url}/api/generate',
                                 json=payload, timeout=self._vlm_timeout)
            resp.raise_for_status()
            return self._parse_json(resp.json().get('response', ''))
        except Exception as e:
            log.warning(f'Ollama call failed: {e}')
            return None

    def _call_claude(self, images_b64: dict) -> dict | None:
        try:
            import anthropic
        except ImportError:
            log.warning('pip install anthropic')
            return None
        try:
            client  = anthropic.Anthropic(api_key=self._api_key)
            content = []
            for cam in ('front', 'back', 'left', 'right'):
                if cam not in images_b64:
                    continue
                content.append({'type': 'text', 'text': f'[{cam.upper()}]'})
                content.append({'type': 'image',
                                'source': {'type': 'base64', 'media_type': 'image/jpeg',
                                           'data': images_b64[cam]}})
            content.append({'type': 'text',
                            'text': ('JSON only, no people: '
                                     '{"landmarks":[{"camera":"front","type":"metal_box",'
                                     '"distance_m":8}],"sticks":[]}')})
            resp = client.messages.create(model=self._model, max_tokens=512,
                                          messages=[{'role': 'user', 'content': content}])
            return self._parse_json(resp.content[0].text)
        except Exception as e:
            log.warning(f'Claude call failed: {e}')
            return None

    @staticmethod
    def _resize_jpeg(b64: str, max_width: int = 640) -> str:
        try:
            import io
            from PIL import Image
            raw = base64.b64decode(b64)
            img = Image.open(io.BytesIO(raw)).convert('RGB')
            w, h = img.size
            if w > max_width:
                img = img.resize((max_width, int(h * max_width / w)), Image.LANCZOS)
            buf = io.BytesIO()
            img.save(buf, 'JPEG', quality=85)
            return base64.b64encode(buf.getvalue()).decode()
        except Exception:
            return b64

    @staticmethod
    def _parse_json(text: str) -> dict | None:
        import json, re
        text = text.strip()
        try:
            return json.loads(text)
        except Exception:
            pass
        m = re.search(r'\{.*\}', text, re.DOTALL)
        if m:
            try:
                return json.loads(m.group())
            except Exception:
                pass
        return None
