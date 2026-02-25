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
OUTPUT_DIR       = os.path.join(current_dir, 'output_glim')
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
    """Transform PointCloud2 into TARGET_FRAME. Returns (N,3) array or None."""
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
    Compose TF from source_frame -> TARGET_FRAME with pose.orientation,
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
        # Quaternion product: q_map = q_tf * q_pose
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
    Render occupancy grid to PNG and save to OUTPUT_DIR/map_latest.png.

    Image convention:
      col = (world_X - u_min) / PIXEL_GRID_SIZE * IMAGE_RES_MUL
      row = (grid_height - 1 - (world_Y - v_min) / PIXEL_GRID_SIZE) * IMAGE_RES_MUL
      np.flipud: image row 0 = world Y_max (north = top)

    Returns (resolution_m_per_px, img_width_px, img_height_px) or (None, None, None).
    """
    try:
        u_min = meta['u_min']
        v_min = meta['v_min']
        gs    = meta['grid_size']
        gh    = meta['grid_height']

        # 0 = unknown (dark), 1 = free (black), 2 = obstacle (white)
        arr = np.full((*occupancy_grid.shape, 3), 40, dtype=np.uint8)
        arr[occupancy_grid == 1] = [0,   0,   0  ]
        arr[occupancy_grid == 2] = [255, 255, 255]

        base = Image.fromarray(np.flipud(arr), mode='RGB')
        img  = base.resize(
            (base.width * IMAGE_RES_MUL, base.height * IMAGE_RES_MUL),
            resample=Image.NEAREST)
        draw = ImageDraw.Draw(img)

        def to_px(wx, wy):
            col = (wx - u_min) / gs * IMAGE_RES_MUL
            row = (gh - 1 - (wy - v_min) / gs) * IMAGE_RES_MUL
            return col, row

        lw = max(int(0.2 * ROBOT_CIRCLE_SIZE), 1)

        # Draw trajectory
        if path_xyz is not None and len(path_xyz) > 1:
            pts_px = [to_px(p[0], p[1]) for p in path_xyz]
            draw.line(pts_px, fill=COLOR_TRAJECTORY, width=lw)

        # Draw robot position and heading arrow
        if path_xyz is not None and len(path_xyz) > 0:
            rx, ry = to_px(path_xyz[-1][0], path_xyz[-1][1])
            off = int(ROBOT_CIRCLE_SIZE * 0.8)
            cx  = rx - off * np.cos(robot_yaw)
            cy  = ry + off * np.sin(robot_yaw)
            al  = int(ROBOT_CIRCLE_SIZE * 1.5)
            ax  = cx + al * np.cos(robot_yaw)
            ay  = cy - al * np.sin(robot_yaw)
            draw.line([(cx, cy), (ax, ay)], fill=COLOR_ARROW, width=lw)
            draw.regular_polygon(
                (ax, ay, ROBOT_CIRCLE_SIZE), 3,
                rotation=np.degrees(robot_yaw) - 90, fill=COLOR_ARROW)
            r = ROBOT_CIRCLE_SIZE
            draw.ellipse([cx-r, cy-r, cx+r, cy+r],
                         fill=COLOR_ROBOT_CIRCLE, outline=COLOR_ARROW, width=lw)

        final_res = float(gs / IMAGE_RES_MUL)
        img.save(os.path.join(OUTPUT_DIR, 'map_latest.png'))

        # Save map metadata (origin, resolution, grid size)
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
    Builds a 2D occupancy map from GLIM's 3D point cloud map.

    odom_callback  (~10 Hz)
        - Receives nav_msgs/Odometry from /glim_ros/odom
        - Accumulates trajectory in TARGET_FRAME
        - Updates current pose (thread-safe)

    pc_callback  (rate-limited to 1/MAP_UPDATE_RATE Hz)
        - Receives PointCloud2 from /glim_ros/map
        - Estimates ground surface via RBF on robot trajectory Z values
        - Builds occupancy grid using relative height above ground
        - Saves PNG + YAML to OUTPUT_DIR

    get_current_pose()
        - Thread-safe accessor; returns {x, y, z, yaw} or None
    """

    def __init__(self):
        super().__init__('continuous_mapper_high_res')

        self._pose_lock    = threading.Lock()
        self._current_pose = None  # {x, y, z, yaw} in TARGET_FRAME

        self.path_positions    = np.empty((0, 3))
        self.latest_pose       = None
        self.latest_pose_frame = None

        # Cached ground surface from last RBF fit
        self._ground_surface = None
        self._ground_gx      = None
        self._ground_gy      = None

        self.last_map_time = None
        self.map_version   = 0

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

        self.pc_sub   = self.create_subscription(PointCloud2, '/glim_ros/map',  self.pc_callback,   map_qos)
        self.odom_sub = self.create_subscription(Odometry,    '/glim_ros/odom', self.odom_callback, odom_qos)
        self.ground_pub = self.create_publisher(PointCloud2, '/ground_from_map', 10)

        print(f'[Mapper] Ready — target frame: "{TARGET_FRAME}"')

    # ── Public API ─────────────────────────────────────────────────────────────

    def get_current_pose(self):
        """Thread-safe. Returns {x, y, z, yaw} in map frame, or None."""
        with self._pose_lock:
            return dict(self._current_pose) if self._current_pose is not None else None

    # ── odom_callback ──────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        """Accumulate robot trajectory from odometry messages."""
        pos = msg.pose.pose.position
        pt  = np.array([[pos.x, pos.y, pos.z]])

        # /glim_ros/odom is already in map frame — no TF needed
        if len(self.path_positions) == 0:
            self.path_positions = pt
        else:
            self.path_positions = np.vstack([self.path_positions, pt])

        self.latest_pose       = msg.pose.pose
        self.latest_pose_frame = msg.header.frame_id

        yaw = extract_yaw_in_map(self.latest_pose, self.latest_pose_frame, self.tf_buffer)

        with self._pose_lock:
            self._current_pose = {
                'x':   float(pt[0, 0]),
                'y':   float(pt[0, 1]),
                'z':   float(pt[0, 2]),
                'yaw': yaw,
            }

    # ── pc_callback ────────────────────────────────────────────────────────────

    def pc_callback(self, msg: PointCloud2):
        """Rate-limited map update. Builds and saves occupancy grid."""
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

        # Remove NaN/Inf
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.size == 0:
            return

        # Clip to path bounding box + margin
        px_min, px_max = self.path_positions[:, 0].min(), self.path_positions[:, 0].max()
        py_min, py_max = self.path_positions[:, 1].min(), self.path_positions[:, 1].max()
        mg   = 50.0
        mask = (
            (pts[:, 0] >= px_min - mg) & (pts[:, 0] <= px_max + mg) &
            (pts[:, 1] >= py_min - mg) & (pts[:, 1] <= py_max + mg)
        )
        pts = pts[mask]
        if pts.size == 0:
            return

        # Estimate ground surface from trajectory Z values
        surface, gx, gy = self._estimate_ground()
        if surface is None:
            return
        self._ground_surface = surface
        self._ground_gx      = gx
        self._ground_gy      = gy

        # Interpolate ground height at each point's (X, Y)
        interp     = RegularGridInterpolator(
            (gy[:, 0], gx[0, :]), surface,
            method='linear', bounds_error=False, fill_value=0)
        ground_z   = interp(np.column_stack([pts[:, 1], pts[:, 0]]))  # (y, x) order
        rel_height = pts[:, 2] - ground_z

        # Build occupancy grid aligned to cell boundaries
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
        grid[iv, iu] = 1   # free space (LiDAR hit, but not obstacle height)

        # Mark obstacles based on relative height above ground
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

        # Optionally publish ground surface as PointCloud2 for RViz visualization
        if PUBLISH_GROUND:
            from std_msgs.msg import Header
            hdr          = Header()
            hdr.stamp    = msg.header.stamp
            hdr.frame_id = TARGET_FRAME
            g_pts = np.vstack([gx.ravel(), gy.ravel(), surface.ravel()]).T
            self.ground_pub.publish(point_cloud2.create_cloud_xyz32(hdr, g_pts))

        self.last_map_time = now
        self.map_version  += 1
        print(f'[Map #{self.map_version}] grid {grid_w}x{grid_h} '
              f'res={final_res:.3f} m/px  yaw={np.degrees(robot_yaw):.1f} deg')

    # ── Ground surface estimation ──────────────────────────────────────────────

    def _estimate_ground(self):
        """
        Fit a thin-plate-spline RBF to robot path Z values.
        Returns (surface_array, gx_mesh, gy_mesh) or (None, None, None).
        The surface represents estimated ground elevation Z = f(X, Y).
        """
        n = len(self.path_positions)
        if n < 10:
            return None, None, None

        # Downsample path to at most 1000 points for RBF speed
        ds      = max(5, n // 1000)
        sampled = self.path_positions[::ds]
        xy      = sampled[:, :2]

        if len(np.unique(xy, axis=0)) < 3:
            return None, None, None

        rbf = RBFInterpolator(xy, sampled[:, 2],
                              kernel='thin_plate_spline', smoothing=0.1)

        mg    = max(RBF_GRID_SIZE * 2, 5.0)
        x_min = xy[:, 0].min() - mg
        x_max = xy[:, 0].max() + mg
        y_min = xy[:, 1].min() - mg
        y_max = xy[:, 1].max() + mg

        # Sanity check: skip if bounding box is unreasonably large
        MAX_RANGE = 5000.0
        if (x_max - x_min) > MAX_RANGE or (y_max - y_min) > MAX_RANGE:
            self.get_logger().warn(
                f'Ground RBF range too large ({x_max-x_min:.0f} x {y_max-y_min:.0f} m)'
                f' — skipping')
            return None, None, None

        xi = np.arange(x_min, x_max + RBF_GRID_SIZE, RBF_GRID_SIZE)
        yi = np.arange(y_min, y_max + RBF_GRID_SIZE, RBF_GRID_SIZE)

        if len(xi) * len(yi) > 10_000_000:
            self.get_logger().warn(f'Ground RBF grid too large ({len(xi)}x{len(yi)}), skipping')
            return None, None, None

        gx, gy = np.meshgrid(xi, yi)
        try:
            surface = rbf(np.column_stack([gx.ravel(), gy.ravel()])).reshape(gx.shape)
            return surface, gx, gy
        except Exception as e:
            self.get_logger().warn(f'RBF failed: {e}')
            return None, None, None


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