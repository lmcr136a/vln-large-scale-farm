import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from sensor_msgs_py import point_cloud2
import numpy as np
import yaml 
import os
from datetime import datetime
from scipy.interpolate import RBFInterpolator, RegularGridInterpolator
from PIL import Image, ImageDraw
import tf2_ros
from tf2_ros import TransformException
from rclpy.duration import Duration

current_dir = os.path.dirname(os.path.abspath(__file__))

# ===== Configuration =====
PUBLISH_GROUND = True
min_z = 0.15
max_z = 1.7
sensor_height = 0.9
output_dir = os.path.join(current_dir, 'output')
pixel_grid_size = 0.3    # Physics/Logic grid size
MIN_POINTS_PER_CELL = 5  # Density filter threshold
IMAGE_RES_MULTIPLIER = 1 # Visual resolution multiplier
update_rate = 1.0
TARGET_FRAME = 'map'     # Target frame for all transformations
RBF_GRID_SIZE = 1.0      # Ground estimation coarse grid

# Dynamic sizing for robot visualization based on grid resolution
ROBOT_ACTUAL_SIZE = 1.2
ROBOT_CIRCLE_SIZE = max(2, int(ROBOT_ACTUAL_SIZE/pixel_grid_size/4*IMAGE_RES_MULTIPLIER))

# Colors (RGB)
COLOR_TRAJECTORY = (100, 255, 170)      
COLOR_ROBOT_CIRCLE = (0, 0, 255)    
COLOR_ARROW = (100, 100, 255)               

os.makedirs(output_dir, exist_ok=True)


def get_projection_basis(path_positions):
    """
    Calculates the average slope of the ground and creates basis vectors 
    for plane projection using the recent path history.
    """
    # Estimate ground normal vector using downsampled path points
    if len(path_positions) < 1000:
        downsampling = 10   # 100
    else:
        downsampling = int(len(path_positions)/100)
        
    pts = path_positions[::downsampling].copy()
    pts[:, 2] -= sensor_height
    centroid = np.mean(pts, axis=0)
    pts_centered = pts - centroid
    _, _, vh = np.linalg.svd(pts_centered)
    normal = vh[2, :]
    if normal[2] < 0: normal = -normal

    # Create basis vectors (u, v) for the plane coordinate system
    z_axis = normal
    x_axis = np.array([1, 0, 0])
    if abs(np.dot(x_axis, z_axis)) > 0.9: x_axis = np.array([0, 1, 0])
    x_axis = x_axis - np.dot(x_axis, z_axis) * z_axis
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    
    return centroid, x_axis, y_axis


def save_high_res_map(occupancy_grid, metadata, path_2d, projected_yaw, output_dir):
    """
    Saves the map as map_latest.png and metadata as map_latest.yaml.
    """
    try:
        # 1. Base map image (Background: g=40)
        img_array = np.zeros((*occupancy_grid.shape, 3), dtype=np.uint8)
        g = 40
        img_array[occupancy_grid == 0] = [g, g, g] 
        img_array[occupancy_grid == 1] = [0, 0, 0]       
        img_array[occupancy_grid == 2] = [255, 255, 255]  # occupied

        original_img = Image.fromarray(np.flipud(img_array), mode='RGB')
        new_size = (original_img.width * IMAGE_RES_MULTIPLIER, original_img.height * IMAGE_RES_MULTIPLIER)
        img = original_img.resize(new_size, resample=Image.NEAREST)
        draw = ImageDraw.Draw(img)

        def to_px(u, v):
            pu = (u - metadata['u_min']) / metadata['grid_size'] * IMAGE_RES_MULTIPLIER
            pv = (metadata['grid_height'] - 1 - (v - metadata['v_min']) / metadata['grid_size']) * IMAGE_RES_MULTIPLIER
            return pu, pv

        line_width = max(int(0.2 * ROBOT_CIRCLE_SIZE), 1)

        # 2. Draw Trajectory
        if len(path_2d) > 1:
            px_path = [to_px(p[0], p[1]) for p in path_2d]
            draw.line(px_path, fill=COLOR_TRAJECTORY, width=line_width)

        # 3. Draw Robot Position & Heading
        curr_u, curr_v = 0.0, 0.0
        if len(path_2d) > 0:
            curr_u, curr_v = path_2d[-1]
            rx, ry = to_px(curr_u, curr_v)
            
            # circle center from robot xy
            circle_offset = int(ROBOT_CIRCLE_SIZE * 0.8)
            cx = rx - circle_offset * np.cos(projected_yaw)
            cy = ry + circle_offset * np.sin(projected_yaw)

            # Heading Arrow
            arrow_len = int(ROBOT_CIRCLE_SIZE * 1.5)
            ax = cx + arrow_len * np.cos(projected_yaw)
            ay = cy - arrow_len * np.sin(projected_yaw)
            
            draw.line([(cx, cy), (ax, ay)], fill=COLOR_ARROW, width=line_width)
            draw.regular_polygon((ax, ay, ROBOT_CIRCLE_SIZE), 3, rotation=np.degrees(projected_yaw)-90, fill=COLOR_ARROW)

            # Position Circle
            r = ROBOT_CIRCLE_SIZE
            draw.ellipse([cx-r, cy-r, cx+r, cy+r], fill=COLOR_ROBOT_CIRCLE, outline=COLOR_ARROW, width=line_width)

        # Save Image as map_latest.png
        img.save(os.path.join(output_dir, 'map_latest.png'))

        # 4. Save Metadata as map_latest.yaml
        yaml_path = os.path.join(output_dir, 'map_latest.yaml')
        yaml_data = {
            'image': 'map_latest.png',
            'resolution': float(metadata['grid_size']),
            'origin': [float(metadata['u_min']), float(metadata['v_min']), 0.0],
            'grid_width': int(metadata['grid_width']),
            'grid_height': int(metadata['grid_height']),
            'robot_x': float(curr_u),
            'robot_y': float(curr_v),
            'robot_yaw': float(projected_yaw),
            'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)

    except Exception as e:
        print(f'[ERROR] map_latest saving failed: {e}')


def transform_pointcloud2_to_map(pc_msg, tf_buffer, target_frame, logger):
    """
    Transform PointCloud2 message to target frame (map).
    Returns transformed points as numpy array or None if transform fails.
    """
    try:
        # Get transform from source frame to target frame
        transform = tf_buffer.lookup_transform(
            target_frame,
            pc_msg.header.frame_id,
            pc_msg.header.stamp,
            timeout=Duration(seconds=0.5)
        )
        
        # Read points from PointCloud2
        gen = point_cloud2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = np.array([(p[0], p[1], p[2]) for p in gen])
        
        if pts.size == 0:
            return None
        
        # Extract translation and rotation from transform
        trans = transform.transform.translation
        rot = transform.transform.rotation
        
        # Convert quaternion to rotation matrix
        x, y, z, w = rot.x, rot.y, rot.z, rot.w
        R = np.array([
            [1-2*(y**2+z**2), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x**2+z**2), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x**2+y**2)]
        ])
        
        # Apply transformation: pts_transformed = R * pts + translation
        pts_transformed = (R @ pts.T).T + np.array([trans.x, trans.y, trans.z])
        
        logger.info(f'Transformed {len(pts_transformed)} points from {pc_msg.header.frame_id} to {target_frame}')
        return pts_transformed
        
    except TransformException as ex:
        logger.warn(f'Could not transform pointcloud: {ex}')
        return None


def transform_path_to_map(path_msg, tf_buffer, target_frame, logger):
    """
    Transform Path message to target frame (map).
    Returns transformed path positions as numpy array or None if transform fails.
    """
    try:
        if not path_msg.poses:
            return None
            
        # Get transform from source frame to target frame
        transform = tf_buffer.lookup_transform(
            target_frame,
            path_msg.header.frame_id,
            path_msg.header.stamp,
            timeout=Duration(seconds=0.5)
        )
        
        # Extract translation and rotation from transform
        trans = transform.transform.translation
        rot = transform.transform.rotation
        
        # Convert quaternion to rotation matrix
        x, y, z, w = rot.x, rot.y, rot.z, rot.w
        R = np.array([
            [1-2*(y**2+z**2), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x**2+z**2), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x**2+y**2)]
        ])
        
        # Transform all poses
        path_positions = np.array([
            [p.pose.position.x, p.pose.position.y, p.pose.position.z] 
            for p in path_msg.poses
        ])
        
        # Apply transformation
        path_transformed = (R @ path_positions.T).T + np.array([trans.x, trans.y, trans.z])
        
        logger.info(f'Transformed path with {len(path_transformed)} poses from {path_msg.header.frame_id} to {target_frame}')
        return path_transformed
        
    except TransformException as ex:
        logger.warn(f'Could not transform path: {ex}')
        return None


def transform_pose_orientation_to_map(pose, source_frame, tf_buffer, target_frame):
    """
    Transform pose orientation to target frame and return the transformed orientation.
    """
    try:
        transform = tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.5)
        )
        
        # Get rotation from transform
        rot_tf = transform.transform.rotation
        x_tf, y_tf, z_tf, w_tf = rot_tf.x, rot_tf.y, rot_tf.z, rot_tf.w
        
        # Get rotation from pose
        q = pose.orientation
        x_p, y_p, z_p, w_p = q.x, q.y, q.z, q.w
        
        # Quaternion multiplication: q_result = q_transform * q_pose
        w_out = w_tf * w_p - x_tf * x_p - y_tf * y_p - z_tf * z_p
        x_out = w_tf * x_p + x_tf * w_p + y_tf * z_p - z_tf * y_p
        y_out = w_tf * y_p - x_tf * z_p + y_tf * w_p + z_tf * x_p
        z_out = w_tf * z_p + x_tf * y_p - y_tf * x_p + z_tf * w_p
        
        # Create transformed orientation
        class Orientation:
            def __init__(self, x, y, z, w):
                self.x, self.y, self.z, self.w = x, y, z, w
        
        return Orientation(x_out, y_out, z_out, w_out)
        
    except TransformException:
        return pose.orientation


class ContinuousPointCloudMapper(Node):
    def __init__(self):
        super().__init__('continuous_mapper_high_res')
        self.path_positions = []
        self.latest_pose = None
        self.latest_pose_frame = None
        self.last_save_time = None
        self.save_count = 0
        
        # Cached projection basis
        self.cached_centroid = None
        self.cached_basis_x = None
        self.cached_basis_y = None
        self.last_basis_update_path_length = 0
        
        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        
        self.pc_sub = self.create_subscription(PointCloud2, '/Laser_map', self.pc_callback, qos)
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, qos)
        self.ground_pub = self.create_publisher(PointCloud2, '/ground_from_map', 10)

        print(f'[INFO] High-resolution Ground Projection Mapper Ready (Target Frame: {TARGET_FRAME})')
    
    def should_update_basis(self):
        """Check if projection basis should be recalculated."""
        current_length = len(self.path_positions)
        
        # 초기 계산 또는 path가 충분히 증가했을 때만 업데이트
        if self.cached_centroid is None:
            return True
        
        # Path가 100개 이상 증가했을 때만 재계산 (약 10m 이동)
        if current_length - self.last_basis_update_path_length > 100:
            return True
        
        return False
    
    def update_projection_basis(self):
        """Update cached projection basis."""
        centroid, basis_x, basis_y = get_projection_basis(self.path_positions)
        self.cached_centroid = centroid
        self.cached_basis_x = basis_x
        self.cached_basis_y = basis_y
        self.last_basis_update_path_length = len(self.path_positions)
        self.get_logger().info(f'Updated projection basis at path length {self.last_basis_update_path_length}')
        
        
    def generate_smooth_ground(self, x_range, y_range):
        """Generates a smooth ground surface using RBF interpolation."""
        if len(self.path_positions) < 10: 
            return None, None, None
            
        # Downsampling
        if len(self.path_positions) < 1000:
            downsampling = 5
        else:
            downsampling = int(len(self.path_positions) / 1000)
        
        sampled_path = self.path_positions[::downsampling]
        sensor_xy = sampled_path[:, 0:2]
        
        if len(np.unique(sensor_xy, axis=0)) < 3: 
            return None, None, None

        ground_z_ref = sampled_path[:, 2] - sensor_height
        rbf = RBFInterpolator(sensor_xy, ground_z_ref, kernel='thin_plate_spline', smoothing=0.1)
        
        # RBF용 coarse grid
        xi = np.arange(x_range[0], x_range[1] + RBF_GRID_SIZE, RBF_GRID_SIZE)
        yi = np.arange(y_range[0], y_range[1] + RBF_GRID_SIZE, RBF_GRID_SIZE)
        gx, gy = np.meshgrid(xi, yi)
        
        try:
            surface = rbf(np.column_stack([gx.ravel(), gy.ravel()])).reshape(gx.shape)
            return surface, gx, gy
        except: 
            return None, None, None
        
    def path_callback(self, msg):
        """Updates robot path and the latest pose (transformed to map frame)."""
        if not msg.poses: 
            return
        
        # Transform path to map frame
        path_transformed = transform_path_to_map(msg, self.tf_buffer, TARGET_FRAME, self.get_logger())
        
        if path_transformed is not None:
            self.path_positions = path_transformed
            self.latest_pose = msg.poses[-1].pose
            self.latest_pose_frame = msg.header.frame_id
        else:
            # Fallback: use original data if transform fails
            self.get_logger().warn(f'Using original path data (frame: {msg.header.frame_id})')
            self.path_positions = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses])
            self.latest_pose = msg.poses[-1].pose
            self.latest_pose_frame = msg.header.frame_id
            
            
    def pc_callback(self, msg):
        """Main callback for processing PointCloud and updating the map."""
        current_time = self.get_clock().now()
        if self.last_save_time is not None:
            if (current_time - self.last_save_time).nanoseconds / 1e9 < update_rate: 
                return

        # Transform pointcloud to map frame
        pts = transform_pointcloud2_to_map(msg, self.tf_buffer, TARGET_FRAME, self.get_logger())
        
        if pts is None or pts.size == 0:
            self.get_logger().warn('PointCloud transformation failed or empty')
            return
            
        if len(self.path_positions) < 15 or self.latest_pose is None: 
            return

        # === 데이터 검증: NaN/inf 제거 ===
        valid_mask = np.isfinite(pts).all(axis=1)
        pts = pts[valid_mask]
        
        if pts.size == 0:
            self.get_logger().warn('No valid points after filtering NaN/inf')
            return
        
        # === Path 기반 범위 계산 (margin 추가) ===
        path_x_min, path_x_max = self.path_positions[:, 0].min(), self.path_positions[:, 0].max()
        path_y_min, path_y_max = self.path_positions[:, 1].min(), self.path_positions[:, 1].max()
        
        margin = 50.0
        x_range = (path_x_min - margin, path_x_max + margin)
        y_range = (path_y_min - margin, path_y_max + margin)
        
        # Pointcloud를 합리적인 범위로 필터링
        in_range = (
            (pts[:, 0] >= x_range[0]) & (pts[:, 0] <= x_range[1]) &
            (pts[:, 1] >= y_range[0]) & (pts[:, 1] <= y_range[1])
        )
        pts = pts[in_range]
        
        if pts.size == 0:
            self.get_logger().warn('No points within valid range')
            return

        # 1. RBF surface estimation using path-based range
        surface, gx, gy = self.generate_smooth_ground(x_range, y_range)
        if surface is None: 
            return

        # 2. Update projection basis only when necessary
        if self.should_update_basis():
            self.update_projection_basis()
        
        centroid = self.cached_centroid
        basis_x = self.cached_basis_x
        basis_y = self.cached_basis_y
        
        # 3. Project coordinates (World -> Plane)
        pts_rel = pts - centroid
        pts_u = np.dot(pts_rel, basis_x)
        pts_v = np.dot(pts_rel, basis_y)
        
        path_rel = self.path_positions - centroid
        path_u = np.dot(path_rel, basis_x)
        path_v = np.dot(path_rel, basis_y)
        path_2d = np.column_stack([path_u, path_v])
        
        # 4. Project robot heading (transform orientation to map frame)
        q_transformed = transform_pose_orientation_to_map(
            self.latest_pose, 
            self.latest_pose_frame, 
            self.tf_buffer, 
            TARGET_FRAME
        )
        
        f_world = np.array([
            1-2*(q_transformed.y**2+q_transformed.z**2), 
            2*(q_transformed.x*q_transformed.y+q_transformed.w*q_transformed.z), 
            2*(q_transformed.x*q_transformed.z-q_transformed.w*q_transformed.y)
        ])
        p_yaw = np.arctan2(np.dot(f_world, basis_y), np.dot(f_world, basis_x))

        # 5. Create Grid (Based on projected u, v coordinates - INCLUDING PATH)
        u_min = min(pts_u.min(), path_u.min())
        u_max = max(pts_u.max(), path_u.max())
        v_min = min(pts_v.min(), path_v.min())
        v_max = max(pts_v.max(), path_v.max())
        width = int(np.ceil((u_max - u_min) / pixel_grid_size))
        height = int(np.ceil((v_max - v_min) / pixel_grid_size))
        grid = np.zeros((height, width), dtype=np.uint8)

        # 6. Obstacle detection using interpolated RBF surface
        iu = np.clip(((pts_u - u_min) / pixel_grid_size).astype(int), 0, width - 1)
        iv = np.clip(((pts_v - v_min) / pixel_grid_size).astype(int), 0, height - 1)
        
        # Create interpolator for RBF surface
        x_coords = gx[0, :]  # X coordinates from meshgrid
        y_coords = gy[:, 0]  # Y coordinates from meshgrid
        interp = RegularGridInterpolator(
            (y_coords, x_coords), 
            surface, 
            method='linear', 
            bounds_error=False, 
            fill_value=0
        )
        
        # Query ground heights at actual point locations
        query_points = np.column_stack([pts[:, 1], pts[:, 0]])
        ground_heights = interp(query_points)
        relative_height = pts[:, 2] - ground_heights

        grid[iv, iu] = 1 # Mark as free space
        obs_candidates = (relative_height >= min_z) & (relative_height <= max_z)
        
        if np.any(obs_candidates):
            o_iu, o_iv = iu[obs_candidates], iv[obs_candidates]
            flat_idx = o_iv * width + o_iu
            counts = np.bincount(flat_idx, minlength=width * height)
            valid_obs = np.where(counts >= MIN_POINTS_PER_CELL)[0]
            grid[valid_obs // width, valid_obs % width] = 2 # Mark as occupied

        # 7. Save map and publish visuals
        self.save_count += 1
        meta = {'u_min': u_min, 'v_min': v_min, 'grid_size': pixel_grid_size, 
                'grid_width': width, 'grid_height': height}
        save_high_res_map(grid, meta, path_2d, p_yaw, output_dir)

        if PUBLISH_GROUND:
            # Create header for ground pointcloud in map frame
            from std_msgs.msg import Header
            ground_header = Header()
            ground_header.stamp = msg.header.stamp
            ground_header.frame_id = TARGET_FRAME
            
            g_pts = np.vstack([gx.ravel(), gy.ravel(), surface.ravel()]).T
            g_msg = point_cloud2.create_cloud_xyz32(ground_header, g_pts)
            self.ground_pub.publish(g_msg)
            
        self.last_save_time = current_time
        print(f'[UPDATE #{self.save_count}] Map saved using ground projection (frame: {TARGET_FRAME})')

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