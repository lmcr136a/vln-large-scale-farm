import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
import numpy as np
import yaml
import os
import threading
from scipy.ndimage import median_filter, binary_dilation
from PIL import Image, ImageDraw
import tf2_ros
from tf2_ros import TransformException
from rclpy.duration import Duration

current_dir = os.path.dirname(os.path.abspath(__file__))

# ===== Configuration =====
PIXEL_GRID_SIZE    = 0.7    # grid cell size (m)
MAP_UPDATE_RATE    = 1.0    # seconds between map redraws
TARGET_FRAME       = 'map'  # all coordinates expressed in this frame
IMAGE_RES_MUL      = 2      # PNG pixels per grid cell

# --- Per-cell point count threshold ---
MIN_POINTS_CELL    = 1      # min lidar hits to mark cell as observed (black)

# --- Obstacle band (relative to z_low) ---
BAND_LOW           = 0.30   # ignore ground roughness below this offset (m)
BAND_HIGH          = 1.80   # robot body / overhang threshold (m)
MIN_POINTS_BAND    = 50     # min points in band to declare occupancy

# --- Step / slope detection ---
STEP_THRESHOLD     = 0.25   # |z_low - median(z_low)| >= this → step obstacle (m)
MEDIAN_FILTER_SIZE = 7      # kernel size for z_low median filter (cells)
STEP_MIN_PTS       = 10     # min points for reliable step detection

# --- Visualization ---
ROBOT_ACTUAL_SIZE  = 1.2                      # robot footprint diameter (m)
# Robot marker sizes are expressed in WORLD METRES, converted to pixels at render time.
ROBOT_RADIUS_M     = ROBOT_ACTUAL_SIZE / 2.0  # circle radius (m)
ROBOT_ARROW_LEN_M  = 1.5                      # heading arrow length (m)

COLOR_TRAJECTORY   = (100, 255, 170)
COLOR_ROBOT_CIRCLE = (0, 0, 255)
COLOR_ARROW        = (100, 100, 255)

OUTPUT_DIR = os.path.join(current_dir, 'output_glim')
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ── Coordinate helpers ─────────────────────────────────────────────────────────

def _quat_to_rot(qx, qy, qz, qw):
    """Return a 3x3 rotation matrix from a unit quaternion."""
    return np.array([
        [1-2*(qy**2+qz**2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)],
        [2*(qx*qy+qw*qz),   1-2*(qx**2+qz**2), 2*(qy*qz-qw*qx)],
        [2*(qx*qz-qw*qy),   2*(qy*qz+qw*qx),   1-2*(qx**2+qy**2)],
    ])


def _apply_tf(pts_n3, tf_stamped):
    """Apply a TF2 TransformStamped to an (N,3) numpy array."""
    t = tf_stamped.transform.translation
    r = tf_stamped.transform.rotation
    R = _quat_to_rot(r.x, r.y, r.z, r.w)
    return (R @ pts_n3.T).T + np.array([t.x, t.y, t.z])


def transform_cloud_to_map(pc_msg, tf_buffer, logger):
    """Transform a PointCloud2 message into TARGET_FRAME. Returns (N,3) or None."""
    try:
        tf = tf_buffer.lookup_transform(
            TARGET_FRAME, pc_msg.header.frame_id,
            pc_msg.header.stamp, timeout=Duration(seconds=0.5))
        gen = point_cloud2.read_points(pc_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.array([(p[0], p[1], p[2]) for p in gen])
        if pts.size == 0:
            return None
        return _apply_tf(pts, tf)
    except TransformException as ex:
        logger.warn(f'Cloud TF failed: {ex}')
        return None


def extract_yaw_in_map(pose, source_frame, tf_buffer):
    """
    Compose TF (source_frame -> TARGET_FRAME) with pose.orientation,
    then extract yaw around the map Z-axis.
    Falls back to the raw pose quaternion on TF failure.
    """
    try:
        tf = tf_buffer.lookup_transform(
            TARGET_FRAME, source_frame,
            rclpy.time.Time(), timeout=Duration(seconds=0.5))
        r  = tf.transform.rotation
        xt, yt, zt, wt = r.x, r.y, r.z, r.w
        q  = pose.orientation
        xp, yp, zp, wp = q.x, q.y, q.z, q.w
        # Quaternion product: q_map = q_tf * q_pose
        w = wt*wp - xt*xp - yt*yp - zt*zp
        x = wt*xp + xt*wp + yt*zp - zt*yp
        y = wt*yp - xt*zp + yt*wp + zt*xp
        z = wt*zp + xt*yp - yt*xp + zt*wp
    except TransformException:
        q = pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
    return float(np.arctan2(2.0*(w*z + x*y), 1.0 - 2.0*(y**2 + z**2)))


# ── Core obstacle detection ────────────────────────────────────────────────────

def build_occupancy_grid(pts, iu, iv, grid_w, grid_h):
    """
    Build a 2D occupancy grid from an (N,3) point cloud.
    Fully vectorized — no Python loops.

    Step 1 — Per-cell statistics
        Sort all points by (cell_index, z) once.
        z_low  : 10th-percentile z  — robust ground-level estimate
        z_high : maximum z          — used for overhang detection

    Step 2 — Obstacle band count  (count_band)
        Count points in [z_low + BAND_LOW, z_low + BAND_HIGH] per cell.
        Ignores ground returns and high overhangs.

    Step 3 — Step / slope check
        median_filter on z_low map.  Large residual = sudden step.
        Gradual slopes pass through and are NOT flagged.

    Step 4 — Overhang exception
        Body space clear + object above head height → Free.

    Step 5 — Boundary: free cells touching unknown → obstacle.

    Returns  np.int8  (grid_h, grid_w)   -1=Unknown  0=Free  1=Occupied
    """
    num_cells = grid_h * grid_w
    flat      = iv * grid_w + iu

    # ── Step 1 ─────────────────────────────────────────────────────────────────
    sort_idx   = np.lexsort((pts[:, 2], flat))
    s_flat     = flat[sort_idx]
    s_z        = pts[sort_idx, 2]

    cell_count = np.bincount(flat, minlength=num_cells)
    cell_end   = np.cumsum(cell_count)
    cell_start = cell_end - cell_count

    valid     = cell_count >= MIN_POINTS_CELL
    valid_idx = np.where(valid)[0]

    z_low_flat  = np.full(num_cells, np.nan, dtype=np.float32)
    z_high_flat = np.full(num_cells, np.nan, dtype=np.float32)

    if valid_idx.size > 0:
        p10_offset = np.floor(cell_count[valid_idx] * 0.10).astype(int)
        p10_global = cell_start[valid_idx] + p10_offset
        z_low_flat[valid_idx]  = s_z[p10_global]
        z_high_flat[valid_idx] = s_z[cell_end[valid_idx] - 1]

    z_low    = z_low_flat.reshape(grid_h, grid_w)
    z_high   = z_high_flat.reshape(grid_h, grid_w)
    hit_mask = valid.reshape(grid_h, grid_w)

    # ── Step 2 ─────────────────────────────────────────────────────────────────
    z_low_per_pt = z_low_flat[flat]
    in_band = (
        np.isfinite(z_low_per_pt) &
        (pts[:, 2] >= z_low_per_pt + BAND_LOW) &
        (pts[:, 2] <= z_low_per_pt + BAND_HIGH)
    )
    count_band = np.bincount(flat[in_band], minlength=num_cells).reshape(grid_h, grid_w)

    # ── Step 3 ─────────────────────────────────────────────────────────────────
    z_low_filled  = np.where(np.isnan(z_low), 0.0, z_low)
    z_low_smooth  = median_filter(z_low_filled, size=MEDIAN_FILTER_SIZE, mode='nearest')
    step_reliable = (cell_count >= STEP_MIN_PTS).reshape(grid_h, grid_w)
    step_diff     = np.where(step_reliable, np.abs(z_low - z_low_smooth), 0.0)
    step_obstacle = step_reliable & (step_diff >= STEP_THRESHOLD)

    # ── Step 4 ─────────────────────────────────────────────────────────────────
    band_obstacle = count_band >= MIN_POINTS_BAND
    obs_mask      = step_obstacle | band_obstacle

    overhang = (
        hit_mask &
        (count_band < MIN_POINTS_BAND) &
        np.isfinite(z_low) &
        (z_high >= z_low + BAND_HIGH)
    )
    obs_mask &= ~overhang

    # ── Assemble ───────────────────────────────────────────────────────────────
    grid = np.full((grid_h, grid_w), -1, dtype=np.int8)
    grid[hit_mask] = 0
    grid[obs_mask] = 1

    # ── Step 5: free cells touching unknown → obstacle ─────────────────────────
    unknown_dilated = binary_dilation((grid == -1), structure=np.ones((3, 3), dtype=bool))
    grid[(grid == 0) & unknown_dilated] = 1

    return grid


# ── Map rendering ──────────────────────────────────────────────────────────────

def render_and_save(grid, meta, path_xyz, robot_yaw):
    """
    Render the occupancy grid to PNG + YAML sidecar.

    Pixel coordinate mapping (Y-up world → Y-down image):
      After np.flipud, the base image row for world Y = wy is:
        base_row = gh - (wy - v_min) / gs       (note: NOT gh-1, see below)
      In the resized image (* IMAGE_RES_MUL):
        px_row   = (gh - (wy - v_min) / gs) * IMAGE_RES_MUL - 0.5

      The -0.5 centres the point within its resized cell. PIL rounds floats
      to int internally, so passing the float is fine for draw calls.

      Similarly for columns:
        px_col = (wx - u_min) / gs * IMAGE_RES_MUL + 0.5

      Using gh (not gh-1) accounts for the fact that after flipud the
      lowest-Y row (iv=0) maps to image row gh-1, i.e. index gh-1 in
      [0, gh-1], which in the resized image spans rows
      (gh-1)*R .. gh*R-1 whose centre is (gh - 0.5)*R - 0.5 = gh*R - R/2 - 0.5.

    Robot marker sizes are expressed in world metres (ROBOT_RADIUS_M,
    ROBOT_ARROW_LEN_M) and converted to pixels here so they scale
    automatically with PIXEL_GRID_SIZE and IMAGE_RES_MUL.
    """
    try:
        u_min = meta['u_min']
        v_min = meta['v_min']
        gs    = meta['grid_size']
        gh    = meta['grid_height']

        arr = np.full((*grid.shape, 3), 40, dtype=np.uint8)
        arr[grid == 0] = [0,   0,   0  ]
        arr[grid == 1] = [255, 255, 255]

        base = Image.fromarray(np.flipud(arr), mode='RGB')
        img  = base.resize(
            (base.width * IMAGE_RES_MUL, base.height * IMAGE_RES_MUL),
            resample=Image.NEAREST)
        draw = ImageDraw.Draw(img)

        # World → resized-image pixel (float).
        # X: cell centre at  (wx - u_min)/gs + 0.5  cells from left
        #    → pixel = that * IMAGE_RES_MUL  - 0.5
        # Y: after flipud, world Y maps to image row counted from top;
        #    cell centre at  gh - (wy - v_min)/gs - 0.5  cells from top
        #    → pixel = that * IMAGE_RES_MUL  - 0.5
        m = IMAGE_RES_MUL / gs   # metres-to-pixels scale factor

        def to_px(wx, wy):
            col = (wx - u_min) * m + 0.5 * IMAGE_RES_MUL - 0.5
            row = (gh * gs - (wy - v_min)) * m - 0.5 * IMAGE_RES_MUL + 0.5
            return col, row

        # Pixel sizes derived from world-metre constants
        r_px  = ROBOT_RADIUS_M   * m   # circle radius in pixels
        al_px = ROBOT_ARROW_LEN_M * m  # arrow length in pixels
        lw    = max(1, int(r_px * 0.25))

        # Trajectory
        if path_xyz is not None and len(path_xyz) > 1:
            pts_px = [to_px(p[0], p[1]) for p in path_xyz]
            draw.line(pts_px, fill=COLOR_TRAJECTORY, width=lw)

        # Robot circle (at exact robot position) + heading arrow
        if path_xyz is not None and len(path_xyz) > 0:
            rx, ry = to_px(path_xyz[-1][0], path_xyz[-1][1])

            # Forward direction in image space:
            #   world forward = (+cos(yaw), +sin(yaw))
            #   image X = world X  →  +cos(yaw)
            #   image Y = -world Y →  -sin(yaw)
            dx =  np.cos(robot_yaw)
            dy = -np.sin(robot_yaw)

            ax = rx + al_px * dx
            ay = ry + al_px * dy

            draw.line([(rx, ry), (ax, ay)], fill=COLOR_ARROW, width=lw)
            draw.regular_polygon(
                (ax, ay, max(2, r_px * 0.6)), 3,
                rotation=np.degrees(robot_yaw) - 90, fill=COLOR_ARROW)
            draw.ellipse(
                [rx - r_px, ry - r_px, rx + r_px, ry + r_px],
                fill=COLOR_ROBOT_CIRCLE, outline=COLOR_ARROW, width=lw)

        final_res = float(gs / IMAGE_RES_MUL)
        img.save(os.path.join(OUTPUT_DIR, 'map_latest.png'))

        yaml_data = {
            'resolution':  final_res,
            'origin':      [float(u_min), float(v_min), 0.0],
            'grid_width':  meta['grid_width'],
            'grid_height': meta['grid_height'],
        }
        with open(os.path.join(OUTPUT_DIR, 'map_latest.yaml'), 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)

        return final_res, img.width, img.height

    except Exception as e:
        print(f'[ERROR] render_and_save: {e}')
        return None, None, None


# ── ROS2 Node ──────────────────────────────────────────────────────────────────

class ContinuousPointCloudMapper(Node):
    """
    2D occupancy mapper consuming GLIM's accumulated 3D point-cloud map.

    Topics
    ------
    /glim_ros/map  (PointCloud2, TRANSIENT_LOCAL) — full accumulated GLIM map
    /glim_ros/odom (Odometry)                     — robot pose in map frame

    Update pipeline  (rate-limited to MAP_UPDATE_RATE Hz)
    -------------------------------------------------------
    1. Transform cloud → TARGET_FRAME.
    2. Compute grid bounds from points + trajectory.
    3. build_occupancy_grid() → int8 grid (-1/0/1).
    4. render_and_save() → PNG + YAML.
    """

    def __init__(self):
        super().__init__('continuous_mapper_high_res')

        self._pose_lock    = threading.Lock()
        self._current_pose = None

        self.path_positions    = np.empty((0, 3))
        self.latest_pose       = None
        self.latest_pose_frame = None
        self.last_map_time     = None
        self.map_version       = 0

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.pc_sub   = self.create_subscription(
            PointCloud2, '/glim_ros/map',  self.pc_callback,   map_qos)
        self.odom_sub = self.create_subscription(
            Odometry,    '/glim_ros/odom', self.odom_callback, odom_qos)

        print(f'[Mapper] Ready — frame="{TARGET_FRAME}"  cell={PIXEL_GRID_SIZE}m  '
              f'band=[{BAND_LOW},{BAND_HIGH}]m  step={STEP_THRESHOLD}m')

    def get_current_pose(self):
        """Thread-safe. Returns {x, y, z, yaw} or None."""
        with self._pose_lock:
            return dict(self._current_pose) if self._current_pose is not None else None

    def odom_callback(self, msg: Odometry):
        """Accumulate robot trajectory. /glim_ros/odom is already in map frame."""
        pos = msg.pose.pose.position
        pt  = np.array([[pos.x, pos.y, pos.z]])

        self.path_positions = pt if len(self.path_positions) == 0 \
            else np.vstack([self.path_positions, pt])

        self.latest_pose       = msg.pose.pose
        self.latest_pose_frame = msg.header.frame_id

        yaw = extract_yaw_in_map(self.latest_pose, self.latest_pose_frame, self.tf_buffer)
        with self._pose_lock:
            self._current_pose = {
                'x': float(pt[0, 0]), 'y': float(pt[0, 1]),
                'z': float(pt[0, 2]), 'yaw': yaw,
            }

    def pc_callback(self, msg: PointCloud2):
        """Rate-limited map update: transform → grid → render → save."""
        now = self.get_clock().now()
        if self.last_map_time is not None:
            if (now - self.last_map_time).nanoseconds / 1e9 < MAP_UPDATE_RATE:
                return
        if len(self.path_positions) < 15 or self.latest_pose is None:
            return

        pts = transform_cloud_to_map(msg, self.tf_buffer, self.get_logger())
        if pts is None or pts.size == 0:
            return
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.size == 0:
            return

        all_x = np.concatenate([pts[:, 0], self.path_positions[:, 0]])
        all_y = np.concatenate([pts[:, 1], self.path_positions[:, 1]])
        u_min  = float(np.floor(all_x.min() / PIXEL_GRID_SIZE) * PIXEL_GRID_SIZE)
        v_min  = float(np.floor(all_y.min() / PIXEL_GRID_SIZE) * PIXEL_GRID_SIZE)
        u_max  = float(np.ceil( all_x.max() / PIXEL_GRID_SIZE) * PIXEL_GRID_SIZE)
        v_max  = float(np.ceil( all_y.max() / PIXEL_GRID_SIZE) * PIXEL_GRID_SIZE)
        grid_w = int(round((u_max - u_min) / PIXEL_GRID_SIZE))
        grid_h = int(round((v_max - v_min) / PIXEL_GRID_SIZE))

        iu = np.clip(((pts[:, 0] - u_min) / PIXEL_GRID_SIZE).astype(int), 0, grid_w - 1)
        iv = np.clip(((pts[:, 1] - v_min) / PIXEL_GRID_SIZE).astype(int), 0, grid_h - 1)

        grid = build_occupancy_grid(pts, iu, iv, grid_w, grid_h)

        robot_yaw = extract_yaw_in_map(
            self.latest_pose, self.latest_pose_frame, self.tf_buffer)

        meta = {
            'u_min': u_min, 'v_min': v_min,
            'grid_size': PIXEL_GRID_SIZE,
            'grid_width': grid_w, 'grid_height': grid_h,
        }
        final_res, _, _ = render_and_save(grid, meta, self.path_positions, robot_yaw)
        if final_res is None:
            return

        self.last_map_time = now
        self.map_version  += 1
        n_obs  = int((grid == 1).sum())
        n_free = int((grid == 0).sum())
        print(f'[Map #{self.map_version}] {grid_w}x{grid_h}  '
              f'free={n_free}  obs={n_obs}  '
              f'res={final_res:.3f}m/px  yaw={np.degrees(robot_yaw):.1f}°')


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    mapper = ContinuousPointCloudMapper()
    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        pass
    finally:
        mapper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()