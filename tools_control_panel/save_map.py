import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
from sensor_msgs_py import point_cloud2
import numpy as np
import yaml
import os
import threading
from datetime import datetime
from scipy.interpolate import RBFInterpolator, RegularGridInterpolator
from PIL import Image, ImageDraw
import tf2_ros
from tf2_ros import TransformException
from rclpy.duration import Duration

current_dir = os.path.dirname(os.path.abspath(__file__))

# ===== Configuration =====
PUBLISH_GROUND   = True
MIN_OBS_Z        = 0.15        # min relative height above ground to be obstacle (m)
MAX_OBS_Z        = 1.7         # max relative height above ground to be obstacle (m)
OUTPUT_DIR       = os.path.join(current_dir, 'output')
PIXEL_GRID_SIZE  = 0.2         # grid cell size in metres
MIN_POINTS_CELL  = 5           # min lidar hits per cell to mark as obstacle
IMAGE_RES_MUL    = 2           # PNG pixels per grid cell
MAP_UPDATE_RATE  = 1.0         # seconds between full map redraws
TARGET_FRAME     = 'map'       # unified 3D coordinate frame for everything
RBF_GRID_SIZE    = 1.0         # ground RBF coarse grid resolution (m)

ROBOT_ACTUAL_SIZE = 1.2
ROBOT_CIRCLE_SIZE = max(2, int(ROBOT_ACTUAL_SIZE / PIXEL_GRID_SIZE / 4 * IMAGE_RES_MUL))

COLOR_TRAJECTORY   = (100, 255, 170)
COLOR_ROBOT_CIRCLE = (0, 0, 255)
COLOR_ARROW        = (100, 100, 255)

os.makedirs(OUTPUT_DIR, exist_ok=True)


# ── Coordinate helpers ─────────────────────────────────────────────────────────

def _quat_to_rot(qx, qy, qz, qw):
    """Return 3x3 rotation matrix from unit quaternion."""
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
    """Transform PointCloud2 into TARGET_FRAME. Returns (N,3) or None."""
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


def transform_path_to_map(path_msg, tf_buffer, logger):
    """Transform all Path poses into TARGET_FRAME. Returns (N,3) or None."""
    try:
        if not path_msg.poses:
            return None
        tf = tf_buffer.lookup_transform(
            TARGET_FRAME, path_msg.header.frame_id,
            path_msg.header.stamp, timeout=Duration(seconds=0.5))
        pts = np.array([
            [p.pose.position.x, p.pose.position.y, p.pose.position.z]
            for p in path_msg.poses
        ])
        return _apply_tf(pts, tf)
    except TransformException as ex:
        logger.warn(f'Path TF failed: {ex}')
        return None


def extract_yaw_in_map(pose, source_frame, tf_buffer):
    """
    Compose the TF from source_frame to TARGET_FRAME with pose.orientation,
    then extract yaw (rotation around map Z-axis).
    Falls back to raw pose quaternion on TF failure.
    """
    try:
        tf = tf_buffer.lookup_transform(
            TARGET_FRAME, source_frame,
            rclpy.time.Time(), timeout=Duration(seconds=0.5))
        r = tf.transform.rotation
        xt, yt, zt, wt = r.x, r.y, r.z, r.w
        q = pose.orientation
        xp, yp, zp, wp = q.x, q.y, q.z, q.w
        # Quaternion product q_map = q_tf * q_pose
        w = wt*wp - xt*xp - yt*yp - zt*zp
        x = wt*xp + xt*wp + yt*zp - zt*yp
        y = wt*yp - xt*zp + yt*wp + zt*xp
        z = wt*zp + xt*yp - yt*xp + zt*wp
    except TransformException:
        q = pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
    return float(np.arctan2(2.0*(w*z + x*y), 1.0 - 2.0*(y**2 + z**2)))


# ── Map rendering ──────────────────────────────────────────────────────────────

def render_and_save(occupancy_grid, meta, path_xyz, robot_yaw):
    """
    Render occupancy grid to PNG and write to OUTPUT_DIR/map_latest.png.

    Image convention:
      - col = (world_X - u_min) / PIXEL_GRID_SIZE * IMAGE_RES_MUL
      - row = (grid_height - 1 - (world_Y - v_min) / PIXEL_GRID_SIZE) * IMAGE_RES_MUL
      - np.flipud so image row 0 = world Y_max  (north = top of image)

    Returns (final_resolution_m_per_px, img_width_px, img_height_px) or (None,None,None).
    """
    try:
        u_min = meta['u_min']
        v_min = meta['v_min']
        gs    = meta['grid_size']
        gh    = meta['grid_height']

        arr = np.full((*occupancy_grid.shape, 3), 40, dtype=np.uint8)
        arr[occupancy_grid == 1] = [0,   0,   0  ]   # free
        arr[occupancy_grid == 2] = [255, 255, 255]   # obstacle

        base = Image.fromarray(np.flipud(arr), mode='RGB')
        img  = base.resize((base.width * IMAGE_RES_MUL, base.height * IMAGE_RES_MUL),
                           resample=Image.NEAREST)
        draw = ImageDraw.Draw(img)

        def to_px(wx, wy):
            col = (wx - u_min) / gs * IMAGE_RES_MUL
            row = (gh - 1 - (wy - v_min) / gs) * IMAGE_RES_MUL
            return col, row

        lw = max(int(0.2 * ROBOT_CIRCLE_SIZE), 1)

        if path_xyz is not None and len(path_xyz) > 1:
            pts_px = [to_px(p[0], p[1]) for p in path_xyz]
            draw.line(pts_px, fill=COLOR_TRAJECTORY, width=lw)

        if path_xyz is not None and len(path_xyz) > 0:
            rx, ry = to_px(path_xyz[-1][0], path_xyz[-1][1])
            off = int(ROBOT_CIRCLE_SIZE * 0.8)
            cx  = rx - off * np.cos(robot_yaw)
            cy  = ry + off * np.sin(robot_yaw)
            al  = int(ROBOT_CIRCLE_SIZE * 1.5)
            ax  = cx + al * np.cos(robot_yaw)
            ay  = cy - al * np.sin(robot_yaw)
            draw.line([(cx, cy), (ax, ay)], fill=COLOR_ARROW, width=lw)
            draw.regular_polygon((ax, ay, ROBOT_CIRCLE_SIZE), 3,
                                 rotation=np.degrees(robot_yaw)-90, fill=COLOR_ARROW)
            r = ROBOT_CIRCLE_SIZE
            draw.ellipse([cx-r, cy-r, cx+r, cy+r],
                         fill=COLOR_ROBOT_CIRCLE, outline=COLOR_ARROW, width=lw)

        final_res = float(gs / IMAGE_RES_MUL)
        img.save(os.path.join(OUTPUT_DIR, 'map_latest.png'))

        # Save map metadata only — robot pose is NOT stored here.
        # Robot pose is sent directly to the web by autonomous_mode.py at high frequency.
        import yaml as _yaml
        yaml_data = {
            'resolution':  final_res,
            'origin':      [float(u_min), float(v_min), 0.0],
            'grid_width':  meta["grid_width"],
            'grid_height': meta["grid_height"],
        }
        with open(os.path.join(OUTPUT_DIR, 'map_latest.yaml'), 'w') as _f:
            _yaml.dump(yaml_data, _f, default_flow_style=False)

        return final_res, img.width, img.height

    except Exception as e:
        print(f'[ERROR] render_and_save: {e}')
        return None, None, None


# ── ROS2 Node ──────────────────────────────────────────────────────────────────

class ContinuousPointCloudMapper(Node):
    """
    Unified 3D map frame node.

    path_callback  (~10 Hz, LiDAR rate)
        - Transforms /path to TARGET_FRAME (3D)
        - Extracts yaw directly from map-frame quaternion
        - Emits robot_pose immediately via socketio  → autonomous driving uses this

    pc_callback  (rate-limited to 1/MAP_UPDATE_RATE Hz)
        - Transforms /Laser_map to TARGET_FRAME
        - Estimates ground surface in 3D via RBF (stored for obstacle height comparison)
        - Builds occupancy grid from relative height above ground surface
        - Saves PNG, emits map_updated metadata via socketio  → client fetches PNG over HTTP

    get_current_pose()
        - Thread-safe accessor; returns {x, y, z, yaw} or None
        - Used by AutonomousController at control loop rate
    """

    def __init__(self):
        super().__init__('continuous_mapper_high_res')


        self._pose_lock    = threading.Lock()
        self._current_pose = None  # {x, y, z, yaw} in TARGET_FRAME

        self.path_positions    = np.empty((0, 3))
        self.latest_pose       = None
        self.latest_pose_frame = None

        # Cached RBF ground (used in pc_callback for obstacle detection)
        self._ground_surface = None
        self._ground_gx      = None
        self._ground_gy      = None

        self.last_map_time = None
        self.map_version   = 0

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.pc_sub     = self.create_subscription(PointCloud2, '/Laser_map', self.pc_callback, qos)
        self.path_sub   = self.create_subscription(Path, '/path', self.path_callback, qos)
        self.ground_pub = self.create_publisher(PointCloud2, '/ground_from_map', 10)

        print(f'[Mapper] Ready — unified 3D frame: "{TARGET_FRAME}"')

    # ── Public API ─────────────────────────────────────────────────────────────

    def get_current_pose(self):
        """Thread-safe. Returns {x, y, z, yaw} in map frame, or None."""
        with self._pose_lock:
            return dict(self._current_pose) if self._current_pose is not None else None

    # ── path_callback ──────────────────────────────────────────────────────────

    def path_callback(self, msg):
        """High-frequency pose update (~10 Hz). Emits robot_pose via socketio."""
        if not msg.poses:
            return

        path_xyz = transform_path_to_map(msg, self.tf_buffer, self.get_logger())

        if path_xyz is not None:
            self.path_positions    = path_xyz
            self.latest_pose       = msg.poses[-1].pose
            self.latest_pose_frame = msg.header.frame_id
        else:
            self.get_logger().warn('Path TF failed — using raw poses')
            self.path_positions = np.array([
                [p.pose.position.x, p.pose.position.y, p.pose.position.z]
                for p in msg.poses
            ])
            self.latest_pose       = msg.poses[-1].pose
            self.latest_pose_frame = msg.header.frame_id

        if self.latest_pose is None or len(self.path_positions) == 0:
            return

        pos = self.path_positions[-1]  # (x, y, z) in map frame
        yaw = extract_yaw_in_map(self.latest_pose, self.latest_pose_frame, self.tf_buffer)

        pose_dict = {
            'x':   float(pos[0]),
            'y':   float(pos[1]),
            'z':   float(pos[2]),
            'yaw': yaw,
        }

        with self._pose_lock:
            self._current_pose = pose_dict


    # ── pc_callback ────────────────────────────────────────────────────────────

    def pc_callback(self, msg):
        """1 Hz map update. Builds occupancy grid and emits map_updated."""
        now = self.get_clock().now()
        if self.last_map_time is not None:
            elapsed = (now - self.last_map_time).nanoseconds / 1e9
            if elapsed < MAP_UPDATE_RATE:
                return

        if len(self.path_positions) < 15 or self.latest_pose is None:
            return

        pts = transform_cloud_to_map(msg, self.tf_buffer, self.get_logger())
        if pts is None or pts.size == 0:
            return

        # Remove NaN / inf
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.size == 0:
            return

        # Clip to path bounding box + margin
        px_min = self.path_positions[:, 0].min()
        px_max = self.path_positions[:, 0].max()
        py_min = self.path_positions[:, 1].min()
        py_max = self.path_positions[:, 1].max()
        mg     = 50.0
        mask   = (
            (pts[:, 0] >= px_min - mg) & (pts[:, 0] <= px_max + mg) &
            (pts[:, 1] >= py_min - mg) & (pts[:, 1] <= py_max + mg)
        )
        pts = pts[mask]
        if pts.size == 0:
            return

        # Estimate / refresh ground surface
        surface, gx, gy = self._estimate_ground()
        if surface is None:
            return
        self._ground_surface = surface
        self._ground_gx      = gx
        self._ground_gy      = gy

        # Interpolate ground height at each point's (X, Y) location
        interp     = RegularGridInterpolator(
            (gy[:, 0], gx[0, :]), surface,
            method='linear', bounds_error=False, fill_value=0)
        ground_z   = interp(np.column_stack([pts[:, 1], pts[:, 0]]))  # (y, x) order
        rel_height = pts[:, 2] - ground_z   # height above ground surface in 3D map frame

        # Occupancy grid — aligned to grid-cell boundaries for pixel-accurate mapping
        all_x = np.concatenate([pts[:, 0], self.path_positions[:, 0]])
        all_y = np.concatenate([pts[:, 1], self.path_positions[:, 1]])
        u_min = float(np.floor(all_x.min() / PIXEL_GRID_SIZE) * PIXEL_GRID_SIZE)
        v_min = float(np.floor(all_y.min() / PIXEL_GRID_SIZE) * PIXEL_GRID_SIZE)
        u_max = float(np.ceil( all_x.max() / PIXEL_GRID_SIZE) * PIXEL_GRID_SIZE)
        v_max = float(np.ceil( all_y.max() / PIXEL_GRID_SIZE) * PIXEL_GRID_SIZE)

        grid_w = int(round((u_max - u_min) / PIXEL_GRID_SIZE))
        grid_h = int(round((v_max - v_min) / PIXEL_GRID_SIZE))
        grid   = np.zeros((grid_h, grid_w), dtype=np.uint8)

        iu = np.clip(((pts[:, 0] - u_min) / PIXEL_GRID_SIZE).astype(int), 0, grid_w - 1)
        iv = np.clip(((pts[:, 1] - v_min) / PIXEL_GRID_SIZE).astype(int), 0, grid_h - 1)
        grid[iv, iu] = 1   # free space (hit by lidar but not obstacle)

        obs = (rel_height >= MIN_OBS_Z) & (rel_height <= MAX_OBS_Z)
        if np.any(obs):
            flat = iv[obs] * grid_w + iu[obs]
            cnt  = np.bincount(flat, minlength=grid_w * grid_h)
            occ  = np.where(cnt >= MIN_POINTS_CELL)[0]
            grid[occ // grid_w, occ % grid_w] = 2   # obstacle

        robot_yaw = extract_yaw_in_map(
            self.latest_pose, self.latest_pose_frame, self.tf_buffer)

        meta = {
            'u_min': u_min, 'v_min': v_min,
            'grid_size': PIXEL_GRID_SIZE,
            'grid_width': grid_w, 'grid_height': grid_h,
        }

        final_res, img_w, img_h = render_and_save(
            grid, meta, self.path_positions, robot_yaw)

        if final_res is None:
            return

        self.map_version += 1
        rp = self._current_pose or {}


        if PUBLISH_GROUND:
            from std_msgs.msg import Header
            hdr = Header()
            hdr.stamp    = msg.header.stamp
            hdr.frame_id = TARGET_FRAME
            g_pts = np.vstack([gx.ravel(), gy.ravel(), surface.ravel()]).T
            self.ground_pub.publish(point_cloud2.create_cloud_xyz32(hdr, g_pts))

        self.last_map_time = now
        print(f'[Map #{self.map_version}] grid {grid_w}×{grid_h} '
              f'res={final_res:.3f} yaw={np.degrees(robot_yaw):.1f}°')

    # ── Ground surface estimation ──────────────────────────────────────────────

    def _estimate_ground(self):
        """
        Fit a thin-plate-spline RBF to robot path Z values in the 3D map frame.
        The resulting surface Z = f(X, Y) represents estimated ground elevation.
        Stored separately from the occupancy grid; used only for rel_height computation.
        Returns (surface_array, gx_mesh, gy_mesh) or (None, None, None).
        """
        n = len(self.path_positions)
        if n < 10:
            return None, None, None

        ds      = max(5, n // 1000)
        sampled = self.path_positions[::ds]
        xy      = sampled[:, :2]

        if len(np.unique(xy, axis=0)) < 3:
            return None, None, None

        rbf = RBFInterpolator(xy, sampled[:, 2],
                              kernel='thin_plate_spline', smoothing=0.1)

        mg = max(RBF_GRID_SIZE * 2, 5.0)
        x_min, x_max = xy[:, 0].min() - mg, xy[:, 0].max() + mg
        y_min, y_max = xy[:, 1].min() - mg, xy[:, 1].max() + mg

        # Sanity check: reject if the range is unreasonably large
        # (indicates TF fallback used raw camera_init coordinates)
        MAX_RANGE = 5000.0   # metres — any real farm field fits within this
        if (x_max - x_min) > MAX_RANGE or (y_max - y_min) > MAX_RANGE:
            self.get_logger().warn(
                f'Ground RBF range too large ({x_max-x_min:.0f} x {y_max-y_min:.0f} m) '
                f'— likely raw camera_init coords, skipping')
            return None, None, None

        xi = np.arange(x_min, x_max + RBF_GRID_SIZE, RBF_GRID_SIZE)
        yi = np.arange(y_min, y_max + RBF_GRID_SIZE, RBF_GRID_SIZE)

        if len(xi) * len(yi) > 10_000_000:
            self.get_logger().warn(f'Ground RBF grid too large ({len(xi)}×{len(yi)})')
            return None, None, None

        gx, gy = np.meshgrid(xi, yi)
        try:
            surface = rbf(np.column_stack([gx.ravel(), gy.ravel()])).reshape(gx.shape)
            return surface, gx, gy
        except Exception as e:
            self.get_logger().warn(f'RBF failed: {e}')
            return None, None, None


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