import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs_py import point_cloud2
import numpy as np
import json
import os
import threading
from scipy.ndimage import uniform_filter, binary_dilation
from PIL import Image, ImageDraw
import tf2_ros
from tf2_ros import TransformException
from rclpy.duration import Duration

current_dir = os.path.dirname(os.path.abspath(__file__))

PIXEL_GRID_SIZE    = 0.05
MAP_UPDATE_RATE    = 1.0
TARGET_FRAME       = 'map'
IMAGE_RES_MUL      = 2
PATH_APPEND_DIST_M = 0.10

MIN_POINTS_CELL    = 1

BAND_LOW           = 0.30
BAND_HIGH          = 1.80
MIN_POINTS_BAND    = 50

STEP_THRESHOLD     = 0.40
MEDIAN_FILTER_SIZE = 15
STEP_MIN_PTS       = 10

Z_COLOR_RANGE      = 0.05

ROBOT_ACTUAL_SIZE  = 1.2
ROBOT_RADIUS_M     = ROBOT_ACTUAL_SIZE / 2.0
ROBOT_ARROW_LEN_M  = 1.5

COLOR_TRAJECTORY   = (100, 255, 170)
COLOR_ROBOT_CIRCLE = (0, 0, 255)
COLOR_ARROW        = (255, 255, 0)

POINT_SAMPLE_RATIO = 1.0

OUTPUT_DIR = os.path.join(current_dir, 'output_glim')
os.makedirs(OUTPUT_DIR, exist_ok=True)


def _quat_to_rot(qx, qy, qz, qw):
    return np.array([
        [1-2*(qy**2+qz**2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)],
        [2*(qx*qy+qw*qz),   1-2*(qx**2+qz**2), 2*(qy*qz-qw*qx)],
        [2*(qx*qz-qw*qy),   2*(qy*qz+qw*qx),   1-2*(qx**2+qy**2)],
    ])


def _apply_tf(pts_n3, tf_stamped):
    t = tf_stamped.transform.translation
    r = tf_stamped.transform.rotation
    R = _quat_to_rot(r.x, r.y, r.z, r.w)
    return (R @ pts_n3.T).T + np.array([t.x, t.y, t.z])


def transform_cloud_to_map(pc_msg, tf_buffer, logger):
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
    try:
        tf = tf_buffer.lookup_transform(
            TARGET_FRAME, source_frame,
            rclpy.time.Time(), timeout=Duration(seconds=0.5))
        r  = tf.transform.rotation
        xt, yt, zt, wt = r.x, r.y, r.z, r.w
        q  = pose.orientation
        xp, yp, zp, wp = q.x, q.y, q.z, q.w
        w = wt*wp - xt*xp - yt*yp - zt*zp
        x = wt*xp + xt*wp + yt*zp - zt*yp
        y = wt*yp - xt*zp + yt*wp + zt*xp
        z = wt*zp + xt*yp - yt*xp + zt*wp
    except TransformException:
        q = pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
    return float(np.arctan2(2.0*(w*z + x*y), 1.0 - 2.0*(y**2 + z**2)))


def _rotate_xy(pts_n3, angle_rad):
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    out = pts_n3.copy()
    out[:, 0] =  c * pts_n3[:, 0] + s * pts_n3[:, 1]
    out[:, 1] = -s * pts_n3[:, 0] + c * pts_n3[:, 1]
    return out


def find_best_rotation(pts):
    """PCA on XY plane — align longest axis of map points to horizontal."""
    xy = pts[:, :2]
    xy = xy - xy.mean(axis=0)
    cov = np.cov(xy.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    # largest eigenvector = principal axis (longest direction of the map)
    principal = eigvecs[:, np.argmax(eigvals)]
    angle = float(np.arctan2(principal[1], principal[0]))
    print(f'[Mapper] Best rotation (PCA): {np.degrees(angle):.0f}deg')
    return angle


def build_occupancy_grid(pts, iu, iv, grid_w, grid_h):
    num_cells = grid_h * grid_w
    flat      = iv * grid_w + iu

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

    z_low_per_pt = z_low_flat[flat]
    in_band = (
        np.isfinite(z_low_per_pt) &
        (pts[:, 2] >= z_low_per_pt + BAND_LOW) &
        (pts[:, 2] <= z_low_per_pt + BAND_HIGH)
    )
    count_band = np.bincount(flat[in_band], minlength=num_cells).reshape(grid_h, grid_w)

    z_low_filled  = np.where(np.isnan(z_low), 0.0, z_low)
    z_low_smooth  = uniform_filter(z_low_filled, size=MEDIAN_FILTER_SIZE, mode='nearest')
    step_reliable = (cell_count >= STEP_MIN_PTS).reshape(grid_h, grid_w)
    step_diff     = np.where(step_reliable, np.abs(z_low - z_low_smooth), 0.0)
    step_obstacle = step_reliable & (step_diff >= STEP_THRESHOLD)

    band_obstacle = count_band >= MIN_POINTS_BAND
    obs_mask      = step_obstacle | band_obstacle

    overhang = (
        hit_mask &
        (count_band < MIN_POINTS_BAND) &
        np.isfinite(z_low) &
        (z_high >= z_low + BAND_HIGH)
    )
    obs_mask &= ~overhang

    grid = np.full((grid_h, grid_w), -1, dtype=np.int8)
    grid[hit_mask] = 0
    grid[obs_mask] = 1

    z_rel = np.where(hit_mask, (z_low - z_low_smooth).astype(np.float32), np.nan)

    return grid, z_rel


_FREE_ANCHORS = np.array([
    [20,  8,   2],
    [70, 35,  10],
    [110, 60, 20],
], dtype=np.float32)

_OBS_ANCHORS = np.array([
    [15, 50,  15],
    [40, 120, 40],
    [80, 210, 80],
], dtype=np.float32)


def _interp_anchors(t, anchors):
    seg   = np.where(t < 0.5, 0, 1)
    t_seg = np.where(t < 0.5, t / 0.5, (t - 0.5) / 0.5)
    c0 = anchors[seg]
    c1 = anchors[seg + 1]
    return np.clip(c0 + (c1 - c0) * t_seg[:, np.newaxis], 0, 255).astype(np.uint8)


def render_and_save(grid, z_rel, meta, path_xyz, robot_yaw, world_rot_angle=0.0):
    try:
        u_min = meta['u_min']
        v_min = meta['v_min']
        gs    = meta['grid_size']
        gh    = meta['grid_height']

        observed   = (grid >= 0)
        z_rel_safe = np.where(observed & np.isfinite(z_rel), z_rel, 0.0)
        t_full = np.clip(z_rel_safe / (2.0 * Z_COLOR_RANGE) + 0.5, 0.0, 1.0)

        arr = np.full((*grid.shape, 3), 30, dtype=np.uint8)

        free_mask = (grid == 0)
        if free_mask.any():
            arr[free_mask] = _interp_anchors(t_full[free_mask], _FREE_ANCHORS)

        obs_mask_2d = (grid == 1)
        if obs_mask_2d.any():
            arr[obs_mask_2d] = _interp_anchors(t_full[obs_mask_2d], _OBS_ANCHORS)

        base = Image.fromarray(np.flipud(arr), mode='RGB')
        img  = base.resize(
            (base.width * IMAGE_RES_MUL, base.height * IMAGE_RES_MUL),
            resample=Image.NEAREST)
        draw = ImageDraw.Draw(img)

        m = IMAGE_RES_MUL / gs

        def to_px(wx, wy):
            col = (wx - u_min) * m + 0.5 * IMAGE_RES_MUL - 0.5
            row = (gh * gs - (wy - v_min)) * m - 0.5 * IMAGE_RES_MUL + 0.5
            return col, row

        r_px  = ROBOT_RADIUS_M   * m
        al_px = ROBOT_ARROW_LEN_M * m
        lw    = max(2, int(r_px * 0.25))

        if path_xyz is not None and len(path_xyz) > 1:
            pts_px = [to_px(p[0], p[1]) for p in path_xyz]
            draw.line(pts_px, fill=COLOR_TRAJECTORY, width=lw)

        if path_xyz is not None and len(path_xyz) > 0:
            rx, ry = to_px(path_xyz[-1][0], path_xyz[-1][1])

            dx =  np.cos(robot_yaw)
            dy = -np.sin(robot_yaw)

            ax = rx + al_px * dx
            ay = ry + al_px * dy

            draw.line([(rx, ry), (ax, ay)], fill=COLOR_ARROW, width=lw)
            draw.ellipse(
                [rx - r_px, ry - r_px, rx + r_px, ry + r_px],
                fill=COLOR_ROBOT_CIRCLE, outline=COLOR_ARROW, width=lw)

        AXIS_MARGIN  = 14
        AXIS_LEN     = 40
        AXIS_LW      = 3
        AXIS_TIP_R   = 4

        ax_ox = img.width  - AXIS_MARGIN
        ax_oy = img.height - AXIS_MARGIN

        th = world_rot_angle
        cos_th, sin_th = float(np.cos(th)), float(np.sin(th))

        x_tip = (ax_ox + AXIS_LEN * cos_th,
                 ax_oy + AXIS_LEN * sin_th)
        draw.line([(ax_ox, ax_oy), x_tip], fill=(220, 50, 50), width=AXIS_LW)
        draw.ellipse([x_tip[0]-AXIS_TIP_R, x_tip[1]-AXIS_TIP_R,
                      x_tip[0]+AXIS_TIP_R, x_tip[1]+AXIS_TIP_R],
                     fill=(220, 50, 50))

        y_tip = (ax_ox + AXIS_LEN * sin_th,
                 ax_oy - AXIS_LEN * cos_th)
        draw.line([(ax_ox, ax_oy), y_tip], fill=(50, 210, 80), width=AXIS_LW)
        draw.ellipse([y_tip[0]-AXIS_TIP_R, y_tip[1]-AXIS_TIP_R,
                      y_tip[0]+AXIS_TIP_R, y_tip[1]+AXIS_TIP_R],
                     fill=(50, 210, 80))

        draw.ellipse([ax_ox-AXIS_TIP_R, ax_oy-AXIS_TIP_R,
                      ax_ox+AXIS_TIP_R, ax_oy+AXIS_TIP_R],
                     fill=(200, 200, 200))

        final_res = float(gs / IMAGE_RES_MUL)
        img.save(os.path.join(OUTPUT_DIR, 'map_latest.png'))

        return final_res, img.width, img.height

    except Exception as e:
        print(f'[ERROR] render_and_save: {e}')
        return None, None, None


class ContinuousPointCloudMapper(Node):

    def __init__(self):
        super().__init__('continuous_mapper_high_res')

        self._pose_lock    = threading.Lock()
        self._current_pose = None

        self.path_positions    = np.empty((0, 3))
        self.latest_pose       = None
        self.latest_pose_frame = None
        self.last_map_time     = None
        self.map_version       = 0
        self._best_angle       = None
        self._map_params       = None   # written to map_params.json at 5Hz

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
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.pc_sub   = self.create_subscription(
            PointCloud2, '/glim_ros/entire_map',          self.pc_callback,   map_qos)
        self.traj_sub = self.create_subscription(
            Path,        '/glim_ros/localized_trajectory', self.traj_callback, odom_qos)
        # Accept both pose topic conventions. The localizer config in this repo
        # publishes /localized_pose, but some launch setups mirror the pose onto
        # /glim_ros/localized_curr_pose. Listening to both keeps the mapper alive
        # if either naming scheme is active.
        self.pose_sub = self.create_subscription(
            PoseStamped, '/localized_pose', self.pose_callback, pose_qos)
        self.pose_sub_glim = self.create_subscription(
            PoseStamped, '/glim_ros/localized_curr_pose', self.pose_callback, pose_qos)

        self._status_timer = self.create_timer(5.0,  self._print_status)
        self._params_timer = self.create_timer(0.2,  self._write_map_params)  # 5 Hz

        print(f'[Mapper] Ready  frame="{TARGET_FRAME}"  cell={PIXEL_GRID_SIZE}m  '
              f'band=[{BAND_LOW},{BAND_HIGH}]m  step={STEP_THRESHOLD}m')

    def _print_status(self):
        if self.map_version > 0:
            return
        if self.latest_pose is None:
            print('[Mapper] Waiting for /localized_pose or /glim_ros/localized_curr_pose ...')
        else:
            print('[Mapper] Waiting for /glim_ros/entire_map ...')

    def _write_map_params(self):
        """Write map_state.json at 5 Hz so autonomous_mode.py and server stay in sync."""
        if self._map_params is None:
            return
        tmp = os.path.join(OUTPUT_DIR, 'map_state.json.tmp')
        dst = os.path.join(OUTPUT_DIR, 'map_state.json')
        try:
            with open(tmp, 'w') as f:
                json.dump(self._map_params, f)
            os.replace(tmp, dst)   # atomic on POSIX
        except Exception:
            pass

    def get_current_pose(self):
        with self._pose_lock:
            return dict(self._current_pose) if self._current_pose is not None else None

    def traj_callback(self, msg: Path):
        """Receive full trajectory from /glim_ros/localized_trajectory (1 Hz).
        Poses are already in map frame — no TF needed."""
        if not msg.poses:
            return

        # Extract XY positions directly (already in map frame)
        pts = np.array([[p.pose.position.x,
                         p.pose.position.y,
                         p.pose.position.z] for p in msg.poses], dtype=np.float64)
        self.path_positions = pts

        # Latest pose = last element in path
        last = msg.poses[-1]
        self.latest_pose       = last.pose
        self.latest_pose_frame = last.header.frame_id or msg.header.frame_id or TARGET_FRAME

        yaw = extract_yaw_in_map(self.latest_pose, self.latest_pose_frame, self.tf_buffer)
        pos = last.pose.position
        with self._pose_lock:
            self._current_pose = {
                'x': float(pos.x), 'y': float(pos.y),
                'z': float(pos.z), 'yaw': yaw,
            }

    def pose_callback(self, msg: PoseStamped):
        """Fallback live pose stream from GLIM.
        This keeps the mapper alive even when localized_trajectory is missing."""
        self.latest_pose = msg.pose
        self.latest_pose_frame = msg.header.frame_id or TARGET_FRAME

        pos = msg.pose.position
        yaw = extract_yaw_in_map(self.latest_pose, self.latest_pose_frame, self.tf_buffer)

        with self._pose_lock:
            self._current_pose = {
                'x': float(pos.x), 'y': float(pos.y),
                'z': float(pos.z), 'yaw': yaw,
            }

        pt = np.array([[float(pos.x), float(pos.y), float(pos.z)]], dtype=np.float64)
        if len(self.path_positions) == 0:
            self.path_positions = pt
            return

        if np.linalg.norm(self.path_positions[-1, :2] - pt[0, :2]) >= PATH_APPEND_DIST_M:
            self.path_positions = np.vstack([self.path_positions, pt])


    def pc_callback(self, msg: PointCloud2):
        now = self.get_clock().now()
        if self.last_map_time is not None:
            if (now - self.last_map_time).nanoseconds / 1e9 < MAP_UPDATE_RATE:
                return
        if len(self.path_positions) < 1 or self.latest_pose is None:
            return

        pts = transform_cloud_to_map(msg, self.tf_buffer, self.get_logger())
        if pts is None or pts.size == 0:
            return
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.size == 0:
            return

        print(f'[Mapper] Building map ...  (pts={len(pts):,})')

        if POINT_SAMPLE_RATIO < 1.0:
            n = max(1, int(len(pts) * POINT_SAMPLE_RATIO))
            pts = pts[np.random.choice(len(pts), n, replace=False)]

        if self._best_angle is None:
            self._best_angle = find_best_rotation(pts)

        theta = self._best_angle

        pts_r  = _rotate_xy(pts, theta)
        path_r = _rotate_xy(self.path_positions, theta) \
            if len(self.path_positions) > 0 else self.path_positions

        all_x = np.concatenate([pts_r[:, 0], path_r[:, 0]])
        all_y = np.concatenate([pts_r[:, 1], path_r[:, 1]])
        u_min  = float(np.floor(all_x.min() / PIXEL_GRID_SIZE) * PIXEL_GRID_SIZE)
        v_min  = float(np.floor(all_y.min() / PIXEL_GRID_SIZE) * PIXEL_GRID_SIZE)
        u_max  = float(np.ceil( all_x.max() / PIXEL_GRID_SIZE) * PIXEL_GRID_SIZE)
        v_max  = float(np.ceil( all_y.max() / PIXEL_GRID_SIZE) * PIXEL_GRID_SIZE)
        grid_w = int(round((u_max - u_min) / PIXEL_GRID_SIZE))
        grid_h = int(round((v_max - v_min) / PIXEL_GRID_SIZE))

        iu = np.clip(((pts_r[:, 0] - u_min) / PIXEL_GRID_SIZE).astype(int), 0, grid_w - 1)
        iv = np.clip(((pts_r[:, 1] - v_min) / PIXEL_GRID_SIZE).astype(int), 0, grid_h - 1)

        grid, z_rel = build_occupancy_grid(pts_r, iu, iv, grid_w, grid_h)

        robot_yaw = extract_yaw_in_map(
            self.latest_pose, self.latest_pose_frame, self.tf_buffer)
        robot_yaw_rotated = robot_yaw - theta

        meta = {
            'u_min': u_min, 'v_min': v_min,
            'grid_size': PIXEL_GRID_SIZE,
            'grid_width': grid_w, 'grid_height': grid_h,
        }
        final_res, _, _ = render_and_save(
            grid, z_rel, meta, path_r, robot_yaw_rotated, world_rot_angle=theta)
        if final_res is None:
            return

        # Update map_params — written to disk at 5 Hz by _write_map_params timer
        self._map_params = {
            'rot_angle':  float(theta),
            'origin_x':   u_min,
            'origin_y':   v_min,
            'resolution': float(PIXEL_GRID_SIZE / IMAGE_RES_MUL),
            'img_width':  int(grid_w * IMAGE_RES_MUL),
            'img_height': int(grid_h * IMAGE_RES_MUL),
        }

        self.last_map_time = now
        self.map_version  += 1
        n_obs  = int((grid == 1).sum())
        n_free = int((grid == 0).sum())
        print(f'[Map #{self.map_version}] {grid_w}x{grid_h}  '
              f'free={n_free}  obs={n_obs}  '
              f'res={final_res:.3f}m/px  yaw={np.degrees(robot_yaw):.1f}deg  '
              f'rot={np.degrees(theta):.0f}deg')


def main():
    rclpy.init()
    mapper = ContinuousPointCloudMapper()
    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        pass
    finally:
        mapper.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
