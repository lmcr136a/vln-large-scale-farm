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
import yaml
import threading
from scipy.ndimage import uniform_filter
from PIL import Image, ImageDraw
import tf2_ros
from tf2_ros import TransformException
from rclpy.duration import Duration

current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP_UPDATE_RATE    = 5.0
TARGET_FRAME       = 'map'
IMAGE_RES_MUL      = 2
MIN_POINTS_CELL    = 2
MAX_HEIGHT_M       = 1.0   # ignore points above this height from local ground (robot clearance)

Z_COLOR_RANGE      = 0.6

ROBOT_ACTUAL_SIZE  = 0.6
ROBOT_RADIUS_M     = ROBOT_ACTUAL_SIZE / 2.0
ROBOT_ARROW_LEN_M  = 0.46

COLOR_TRAJECTORY   = (100, 255, 170)
COLOR_ROBOT_CIRCLE = (0, 0, 255)
COLOR_ARROW        = (255, 255, 0)
COLOR_GROUND = (255, 140, 0)

POINT_SAMPLE_RATIO = 1.0
PIXEL_GRID_SIZE = 0.5


SMOOTH_RADIUS_M    = 10   # neighbor range (m)
MEDIAN_FILTER_SIZE = max(3, int(SMOOTH_RADIUS_M / PIXEL_GRID_SIZE))


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
    gen = point_cloud2.read_points(pc_msg, field_names=('x', 'y', 'z'), skip_nans=True)
    pts = np.array([(p[0], p[1], p[2]) for p in gen])
    if pts.size == 0:
        return None
    if pc_msg.header.frame_id == TARGET_FRAME:
        return pts
    try:
        tf = tf_buffer.lookup_transform(
            TARGET_FRAME, pc_msg.header.frame_id,
            pc_msg.header.stamp, timeout=Duration(seconds=0.5))
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
    s_z        = pts[sort_idx, 2]

    cell_count = np.bincount(flat, minlength=num_cells)
    cell_end   = np.cumsum(cell_count)
    cell_start = cell_end - cell_count

    valid     = cell_count >= MIN_POINTS_CELL
    valid_idx = np.where(valid)[0]

    z_low_flat = np.full(num_cells, np.nan, dtype=np.float32)
    if valid_idx.size > 0:
        p10_offset = np.floor(cell_count[valid_idx] * 0.10).astype(int)
        p10_global = cell_start[valid_idx] + p10_offset
        z_low_flat[valid_idx] = s_z[p10_global]

    z_low    = z_low_flat.reshape(grid_h, grid_w)
    hit_mask = valid.reshape(grid_h, grid_w)

    z_low_filled = np.where(np.isnan(z_low), 0.0, z_low)
    z_low_smooth = uniform_filter(z_low_filled, size=MEDIAN_FILTER_SIZE, mode='nearest')

    grid  = np.where(hit_mask, 0, -1).astype(np.int8)
    z_rel = np.where(hit_mask, (z_low - z_low_smooth).astype(np.float32), np.nan)

    return grid, z_rel


_COLOR_LOW  = np.array([20,  20,  50],  dtype=np.float32)  # low relative height
_COLOR_HIGH = np.array([255, 200, 200], dtype=np.float32)  # high relative height


def _height_color(t: np.ndarray) -> np.ndarray:
    """t: (N,) float 0-1 → (N, 3) uint8, linear gradient low→high."""
    return np.clip(
        _COLOR_LOW + (_COLOR_HIGH - _COLOR_LOW) * t[:, np.newaxis],
        0, 255
    ).astype(np.uint8)


def render_and_save(grid, z_rel, meta, path_xyz, robot_yaw, output_path, world_rot_angle=0.0, robot_pos=None,
                    ground_xyz=None, traversable_cells=None):
    try:
        u_min = meta['u_min']
        v_min = meta['v_min']
        gs    = meta['grid_size']
        gh    = meta['grid_height']

        observed   = (grid >= 0)
        z_rel_safe = np.where(observed & np.isfinite(z_rel), z_rel, 0.0)
        t_full = np.clip(z_rel_safe / (2.0 * Z_COLOR_RANGE) + 0.5, 0.0, 1.0)

        arr = np.full((*grid.shape, 3), 30, dtype=np.uint8)

        observed = (grid >= 0)
        if observed.any():
            arr[observed] = _height_color(t_full[observed])

        if ground_xyz is not None and len(ground_xyz) > 0:
            gu = ((ground_xyz[:, 0] - u_min) / gs).astype(int)
            gv = ((ground_xyz[:, 1] - v_min) / gs).astype(int)

            inside = (
                (gu >= 0) & (gu < arr.shape[1]) &
                (gv >= 0) & (gv < arr.shape[0])
            )
            overlay_mask = inside
            if traversable_cells is not None:
                traversable_cells = np.asarray(traversable_cells, dtype=bool).reshape(-1)
                expected_cells = arr.shape[0] * arr.shape[1]
                if traversable_cells.size == expected_cells:
                    overlay_mask = np.zeros_like(inside, dtype=bool)
                    inside_idx = np.flatnonzero(inside)
                    flat_idx = gv[inside_idx] * arr.shape[1] + gu[inside_idx]
                    overlay_mask[inside_idx] = traversable_cells[flat_idx]
                else:
                    print('[Mapper] traversable cell mask size mismatch; rendering unfiltered ground overlay')

            alpha = 0.8
            rr = gv[overlay_mask]
            cc = gu[overlay_mask]
            ground_color = np.array(COLOR_GROUND, dtype=np.float32)
            arr[rr, cc] = (
                arr[rr, cc].astype(np.float32) * (1.0 - alpha) +
                ground_color * alpha
            ).astype(np.uint8)

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

        _robot_pos = robot_pos if robot_pos is not None else (path_xyz[-1] if path_xyz is not None and len(path_xyz) > 0 else None)
        if _robot_pos is not None:
            rx, ry = to_px(_robot_pos[0], _robot_pos[1])

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
        img.save(output_path)

        return final_res, img.width, img.height

    except Exception as e:
        print(f'[ERROR] render_and_save: {e}')
        return None, None, None


class ContinuousPointCloudMapper(Node):

    def __init__(self, cfg: dict):
        super().__init__('continuous_mapper_high_res')

        p = cfg['paths']
        t = cfg['ros2']['topics']
        raw_map_dir = os.path.expanduser(p['map_dir'])
        if not os.path.isabs(raw_map_dir):
            raw_map_dir = os.path.join(current_dir, raw_map_dir)
        self._output_dir  = os.path.normpath(raw_map_dir)
        self._map_image   = p.get('map_image',  'map_latest.png')
        self._map_state   = p.get('map_state',  'map_state.json')
        os.makedirs(self._output_dir, exist_ok=True)
        print(f'[Mapper] Output dir: {self._output_dir}')

        self._pose_lock    = threading.Lock()
        self._current_pose = None

        self.path_positions    = np.empty((0, 3))
        self.latest_pose       = None
        self.latest_pose_frame = None
        self.last_map_time     = None
        self.map_version       = 0
        self._best_angle       = None
        self._map_params       = None
        self._use_aligned      = False   # True when entire_map is empty → fall back to aligned_points
        self.latest_ground_pts = None

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

        curr_pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        aligned_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.pc_sub        = self.create_subscription(
            PointCloud2,  t['entire_map'],     self.pc_callback,        map_qos)
        self.aligned_sub   = self.create_subscription(
            PointCloud2,  t['aligned_points'], self.aligned_callback,   aligned_qos)
        self.traj_sub      = self.create_subscription(
            Path,         t['trajectory'],  self.traj_callback,      odom_qos)
        self.curr_pose_sub = self.create_subscription(
            PoseStamped,  t['pose'],        self.curr_pose_callback, curr_pose_qos)
        self.ground_sub = self.create_subscription(
            PointCloud2, t['ground_points'], self.ground_callback, map_qos)

        self._status_timer = self.create_timer(5.0,  self._print_status)
        self._params_timer = self.create_timer(0.2,  self._write_map_params)

        print(f'[Mapper] Ready  frame="{TARGET_FRAME}"  cell={PIXEL_GRID_SIZE}m')

    def _print_status(self):
        if self.map_version > 0:
            return
        if self.latest_pose is None:
            print('[Mapper] Waiting for /localized_pose ...')
        else:
            src = 'aligned_points (fallback)' if self._use_aligned else 'entire_map'
            print(f'[Mapper] Waiting for /glim_ros/{src} ...')

    def _write_map_params(self):
        """Write map_state.json at 5 Hz so autonomous_mode.py and server stay in sync."""
        if self._map_params is None:
            return
        tmp = os.path.join(self._output_dir, self._map_state + '.tmp')
        dst = os.path.join(self._output_dir, self._map_state)
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
        Poses are already in map frame — only update path_positions for drawing."""
        if not msg.poses:
            return
        self.path_positions = np.array([[p.pose.position.x,
                                         p.pose.position.y,
                                         p.pose.position.z] for p in msg.poses], dtype=np.float64)

    def curr_pose_callback(self, msg: PoseStamped):
        """Receive current robot pose from /glim_ros/localized_curr_pose (10 Hz).
        Used for robot drawing and current pose tracking."""
        self.latest_pose       = msg.pose
        self.latest_pose_frame = msg.header.frame_id or TARGET_FRAME

        yaw = extract_yaw_in_map(self.latest_pose, self.latest_pose_frame, self.tf_buffer)
        pos = msg.pose.position
        with self._pose_lock:
            self._current_pose = {
                'x': float(pos.x), 'y': float(pos.y),
                'z': float(pos.z), 'yaw': yaw,
            }


    def pc_callback(self, msg: PointCloud2):
        pts = transform_cloud_to_map(msg, self.tf_buffer, self.get_logger())

        if pts is None or pts.size == 0:
            if not self._use_aligned:
                self._use_aligned = True
                print('[Mapper] entire_map has no points — switching to aligned_points fallback')
            return

        # entire_map has real data; make sure fallback is off
        if self._use_aligned:
            self._use_aligned = False
            print('[Mapper] entire_map restored — disabling aligned_points fallback')

        self._process_cloud(pts)

    def aligned_callback(self, msg: PointCloud2):
        if not self._use_aligned:
            return
        pts = transform_cloud_to_map(msg, self.tf_buffer, self.get_logger())
        if pts is None or pts.size == 0:
            return
        self._process_cloud(pts)
    def ground_callback(self, msg: PointCloud2):
        pts = transform_cloud_to_map(msg, self.tf_buffer, self.get_logger())
        if pts is None or pts.size == 0:
            self.latest_ground_pts = None
            return
        pts = pts[np.isfinite(pts).all(axis=1)]
        self.latest_ground_pts = pts if pts.size > 0 else None
    def _process_cloud(self, pts: np.ndarray):
        now = self.get_clock().now()
        if self.last_map_time is not None:
            if (now - self.last_map_time).nanoseconds / 1e9 < MAP_UPDATE_RATE:
                return
        if self.latest_pose is None:
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

        ground_r = None
        if self.latest_ground_pts is not None:
            ground_r = _rotate_xy(self.latest_ground_pts, theta)

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

        #traversability mask - includes height filter
        cell_idx = iv * grid_w + iu

        z_floor = np.full(grid_w * grid_h, np.inf, dtype=np.float32)
        np.minimum.at(z_floor, cell_idx, pts_r[:, 2].astype(np.float32))

        z_rel = pts_r[:, 2] - z_floor[cell_idx]

        ground_band_pts = (z_rel >= 0.0) & (z_rel <= 0.3)
        blocked_pts = (z_rel > 0.3) & (z_rel < 1.0)

        ground_cells = np.zeros(grid_w * grid_h, dtype=bool)
        blocked_cells = np.zeros(grid_w * grid_h, dtype=bool)

        ground_cells[cell_idx[ground_band_pts]] = True
        blocked_cells[cell_idx[blocked_pts]] = True

        traversable_cells = ground_cells & ~blocked_cells

        # Filter out points above robot clearance height from local ground
        cell_idx   = iv * grid_w + iu
        z_floor    = np.full(grid_w * grid_h, np.inf, dtype=np.float32)
        np.minimum.at(z_floor, cell_idx, pts_r[:, 2].astype(np.float32))
        keep       = pts_r[:, 2] <= z_floor[cell_idx] + MAX_HEIGHT_M
        pts_r, iu, iv = pts_r[keep], iu[keep], iv[keep]

        grid, z_rel = build_occupancy_grid(pts_r, iu, iv, grid_w, grid_h)

        robot_yaw = extract_yaw_in_map(
            self.latest_pose, self.latest_pose_frame, self.tf_buffer)
        robot_yaw_rotated = robot_yaw - theta

        meta = {
            'u_min': u_min, 'v_min': v_min,
            'grid_size': PIXEL_GRID_SIZE,
            'grid_width': grid_w, 'grid_height': grid_h,
        }
        p = self.latest_pose.position
        robot_xy = np.array([[p.x, p.y, 0.0]])
        robot_xy_r = _rotate_xy(robot_xy, theta)
        robot_pos_r = (robot_xy_r[0, 0], robot_xy_r[0, 1])

        final_res, _, _ = render_and_save(
            grid, z_rel, meta, path_r, robot_yaw_rotated,
            os.path.join(self._output_dir, self._map_image),
            world_rot_angle=theta, robot_pos=robot_pos_r,
            ground_xyz=ground_r, traversable_cells=traversable_cells)
        if final_res is None:
            return

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
        if self.map_version == 1:
            print(f'[Mapper] Saving map to: {os.path.join(self._output_dir, self._map_image)}')
        n_obs  = int((grid == 1).sum())
        n_free = int((grid == 0).sum())
        src_label = 'aligned' if self._use_aligned else 'entire'
        print(f'[Map #{self.map_version}] {grid_w}x{grid_h}  '
              f'free={n_free}  obs={n_obs}  '
              f'res={final_res:.3f}m/px  yaw={np.degrees(robot_yaw):.1f}deg  '
              f'rot={np.degrees(theta):.0f}deg  src={src_label}')


def main():
    import argparse
    parser = argparse.ArgumentParser()
    
    default_cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/farm_config.yaml")
    
    parser.add_argument('--config', default=default_cfg)
    args, _ = parser.parse_known_args()
    with open(os.path.expanduser(args.config)) as f:
        cfg = yaml.safe_load(f)

    rclpy.init()
    mapper = ContinuousPointCloudMapper(cfg)
    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        pass
    finally:
        mapper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
