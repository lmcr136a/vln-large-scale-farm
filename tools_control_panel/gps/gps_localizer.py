"""
GpsLocalizer: RTK GPS + Xsens IMU → local ENU PoseStamped.

Publishes /gps_pose (PoseStamped) so the existing AutonomousController works
unchanged — just point ros2.topics.pose to /gps_pose in farm_config.yaml.

ENU convention (ROS2 standard):
  x = East  (metres from origin)
  y = North (metres from origin)
  yaw = 0 → facing East, π/2 → facing North
"""
import json
import logging
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

log = logging.getLogger(__name__)

R_EARTH = 6378137.0

BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def gps_to_enu(lat, lon, origin_lat, origin_lon):
    """lat/lon → local ENU metres (x=East, y=North)."""
    x = R_EARTH * math.radians(lon - origin_lon) * math.cos(math.radians(origin_lat))
    y = R_EARTH * math.radians(lat - origin_lat)
    return x, y


def enu_to_gps(x, y, origin_lat, origin_lon):
    """Local ENU metres → lat/lon."""
    lat = origin_lat + math.degrees(y / R_EARTH)
    lon = origin_lon + math.degrees(x / (R_EARTH * math.cos(math.radians(origin_lat))))
    return lat, lon


def _quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y ** 2 + z ** 2))


class GpsLocalizer(Node):
    def __init__(self, config: dict):
        super().__init__('gps_localizer')
        self._cfg = config
        self._lock = threading.Lock()

        self._origin_lat: float | None = None
        self._origin_lon: float | None = None
        self._current_lat: float | None = None
        self._current_lon: float | None = None
        self._current_yaw: float = 0.0   # from IMU (fallback)
        self._gps_heading: float | None = None  # derived from GPS displacement (preferred)
        self._current_x: float | None = None
        self._current_y: float | None = None

        self._gps_track: list[dict] = []
        self._track_dirty = False

        # Locate data directory relative to project root
        proj_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        data_dir_rel = config.get('paths', {}).get('data_dir', '../data')
        data_dir = os.path.normpath(os.path.join(proj_dir, data_dir_rel))
        os.makedirs(data_dir, exist_ok=True)
        self._origin_file    = os.path.join(data_dir, 'gps_origin.json')
        self._track_file     = os.path.join(data_dir, 'gps_track.json')
        self._past_paths_dir = os.path.join(data_dir, 'past_paths')

        self._load_origin()
        self._load_track()

        topics = config.get('ros2', {}).get('topics', {})
        gps_topic      = topics.get('gps',      '/gps/fix')
        imu_topic      = topics.get('imu',       '/xsens/imu/data')
        pose_out_topic = topics.get('gps_pose',  '/gps_pose')

        self._pub = self.create_publisher(PoseStamped, pose_out_topic, 10)
        self.create_subscription(NavSatFix, gps_topic, self._on_gps, BEST_EFFORT)
        self.create_subscription(Imu,       imu_topic, self._on_imu, BEST_EFFORT)

        # Periodically flush GPS track to disk
        self.create_timer(30.0, self._flush_track)

        self.get_logger().info(f'GpsLocalizer ready → {pose_out_topic}')
        if self._origin_lat is not None:
            self.get_logger().info(
                f'Origin loaded: {self._origin_lat:.7f}, {self._origin_lon:.7f}'
            )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        with self._lock:
            self._current_yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)

    def _on_gps(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        lat, lon = msg.latitude, msg.longitude
        with self._lock:
            self._current_lat = lat
            self._current_lon = lon
            if self._origin_lat is None:
                self._origin_lat = lat
                self._origin_lon = lon
                self._save_origin()
                self.get_logger().info(f'GPS origin auto-set: {lat:.7f}, {lon:.7f}')
            x, y = gps_to_enu(lat, lon, self._origin_lat, self._origin_lon)
            # Derive heading from GPS displacement (reliable even at rosbag start)
            if self._current_x is not None:
                dx = x - self._current_x
                dy = y - self._current_y
                if math.hypot(dx, dy) > 0.1:   # 10 cm — safe for RTK
                    self._gps_heading = math.atan2(dy, dx)
            self._current_x = x
            self._current_y = y
            yaw = self._gps_heading if self._gps_heading is not None else self._current_yaw
            self._gps_track.append({'lat': lat, 'lon': lon, 'x': x, 'y': y})
            self._track_dirty = True

        msg_out = PoseStamped()
        msg_out.header.stamp    = msg.header.stamp
        msg_out.header.frame_id = 'enu'
        msg_out.pose.position.x = x
        msg_out.pose.position.y = y
        msg_out.pose.position.z = 0.0
        msg_out.pose.orientation.x = 0.0
        msg_out.pose.orientation.y = 0.0
        msg_out.pose.orientation.z = math.sin(yaw / 2)
        msg_out.pose.orientation.w = math.cos(yaw / 2)
        self._pub.publish(msg_out)

    # ── Public API ───────────────────────────────────────────────────────────

    def get_current_gps(self) -> tuple | None:
        """Returns (lat, lon, yaw_rad) or None."""
        with self._lock:
            if self._current_lat is None:
                return None
            yaw = self._gps_heading if self._gps_heading is not None else self._current_yaw
            return self._current_lat, self._current_lon, yaw

    def get_current_enu(self) -> tuple | None:
        """Returns (x, y, yaw) in local ENU or None."""
        with self._lock:
            if self._current_x is None:
                return None
            yaw = self._gps_heading if self._gps_heading is not None else self._current_yaw
            return self._current_x, self._current_y, yaw

    def get_origin(self) -> tuple:
        """Returns (origin_lat, origin_lon) or (None, None)."""
        with self._lock:
            return self._origin_lat, self._origin_lon

    def get_track(self) -> list:
        with self._lock:
            return list(self._gps_track)

    def get_map_bounds_enu(self, padding_m: float = 20.0) -> tuple:
        """Returns (x_min, y_min, x_max, y_max) covering all track points."""
        with self._lock:
            if not self._gps_track:
                return -padding_m, -padding_m, padding_m, padding_m
            xs = [p['x'] for p in self._gps_track]
            ys = [p['y'] for p in self._gps_track]
        return (min(xs) - padding_m, min(ys) - padding_m,
                max(xs) + padding_m, max(ys) + padding_m)

    def enu_to_gps(self, x: float, y: float) -> tuple:
        """Local ENU → (lat, lon). Returns (None, None) if origin not set."""
        with self._lock:
            if self._origin_lat is None:
                return None, None
            return enu_to_gps(x, y, self._origin_lat, self._origin_lon)

    def is_heading_valid(self) -> bool:
        """True once the robot has moved enough to compute GPS-derived heading."""
        with self._lock:
            return self._gps_heading is not None

    def reset_for_replay(self):
        """Clear in-memory state for a fresh replay session (keeps persisted origin file)."""
        with self._lock:
            track_snapshot    = list(self._gps_track)
            should_archive    = self._track_dirty   # only archive if new GPS data arrived
            self._gps_track   = []
            self._track_dirty = False
            self._gps_heading = None
            self._current_x   = None
            self._current_y   = None
            self._current_lat = None
            self._current_lon = None
        if should_archive:
            self._archive_track_data(track_snapshot)
        log.info('GpsLocalizer: session reset for replay')

    def get_past_paths(self) -> list:
        """Return up to 20 most recent past paths as list of point lists, oldest first."""
        try:
            if not os.path.isdir(self._past_paths_dir):
                return []
            files = sorted(
                f for f in os.listdir(self._past_paths_dir)
                if f.startswith('track_') and f.endswith('.json')
            )
            paths = []
            for fname in files[-20:]:
                try:
                    with open(os.path.join(self._past_paths_dir, fname)) as f:
                        paths.append(json.load(f))
                except Exception:
                    pass
            return paths
        except Exception:
            return []

    def set_origin(self, lat: float, lon: float):
        """Manually set the ENU origin (re-projects existing track)."""
        with self._lock:
            self._origin_lat = lat
            self._origin_lon = lon
            for p in self._gps_track:
                p['x'], p['y'] = gps_to_enu(p['lat'], p['lon'], lat, lon)
        self._save_origin()
        self.get_logger().info(f'GPS origin manually set: {lat:.7f}, {lon:.7f}')

    # ── Persistence ──────────────────────────────────────────────────────────

    def _load_origin(self):
        try:
            with open(self._origin_file) as f:
                d = json.load(f)
            self._origin_lat = float(d['lat'])
            self._origin_lon = float(d['lon'])
        except FileNotFoundError:
            pass
        except Exception as e:
            log.warning(f'GPS origin load failed: {e}')

    def _save_origin(self):
        try:
            with open(self._origin_file, 'w') as f:
                json.dump({'lat': self._origin_lat, 'lon': self._origin_lon}, f)
        except Exception as e:
            log.warning(f'GPS origin save failed: {e}')

    def _load_track(self):
        try:
            with open(self._track_file) as f:
                self._gps_track = json.load(f)
            log.info(f'Loaded {len(self._gps_track)} GPS track points')
            # Archive this version of the track (dedup by mtime — safe to call every startup)
            self._archive_by_mtime()
        except FileNotFoundError:
            pass
        except Exception as e:
            log.warning(f'GPS track load failed: {e}')

    def _archive_by_mtime(self):
        """Archive gps_track.json to past_paths/ keyed by file mtime (idempotent)."""
        if not os.path.exists(self._track_file):
            return
        try:
            mtime = int(os.path.getmtime(self._track_file))
            os.makedirs(self._past_paths_dir, exist_ok=True)
            archive_path = os.path.join(self._past_paths_dir, f'track_{mtime}.json')
            if os.path.exists(archive_path):
                return  # already archived this version
            if len(self._gps_track) < 10:
                return
            with open(archive_path, 'w') as f:
                json.dump(self._gps_track, f)
            log.info(f'Archived {len(self._gps_track)} GPS track points → past_paths/track_{mtime}.json')
            self._trim_past_paths(keep=20)
        except Exception as e:
            log.warning(f'GPS track archive failed: {e}')

    def _archive_track_data(self, track: list):
        """Archive an in-memory track to past_paths/ with current timestamp."""
        if len(track) < 10:
            return
        try:
            os.makedirs(self._past_paths_dir, exist_ok=True)
            ts = int(time.time())
            archive_path = os.path.join(self._past_paths_dir, f'track_{ts}.json')
            with open(archive_path, 'w') as f:
                json.dump(track, f)
            log.info(f'Archived {len(track)} GPS track points → past_paths/track_{ts}.json')
            self._trim_past_paths(keep=20)
        except Exception as e:
            log.warning(f'GPS track archive failed: {e}')

    def _trim_past_paths(self, keep: int = 20):
        """Delete oldest past_paths files beyond keep count."""
        try:
            files = sorted(
                os.path.join(self._past_paths_dir, f)
                for f in os.listdir(self._past_paths_dir)
                if f.startswith('track_') and f.endswith('.json')
            )
            for fp in files[:-keep]:
                os.remove(fp)
        except Exception as e:
            log.warning(f'GPS past paths trim failed: {e}')

    def _flush_track(self):
        with self._lock:
            if not self._track_dirty:
                return
            self._track_dirty = False
            to_save = self._gps_track[-10000:]  # limit file size only; keep full track in memory
        try:
            with open(self._track_file, 'w') as f:
                json.dump(to_save, f)
        except Exception as e:
            log.warning(f'GPS track save failed: {e}')
