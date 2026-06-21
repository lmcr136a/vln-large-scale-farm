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
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus
from std_msgs.msg import String

log = logging.getLogger(__name__)

R_EARTH = 6378137.0

# Heading-from-GPS is derived from displacement over a short time window rather
# than between two consecutive fixes — a single-step distance check is fooled by
# RTK jitter when the robot is stationary (noise can exceed the step threshold in
# a random direction). Averaging over a longer baseline makes real travel (which
# grows linearly with time) dominate jitter (which doesn't).
HEADING_WINDOW_S  = 0.8   # seconds of GPS history kept for the baseline
HEADING_MIN_DT    = 0.3   # minimum window span before trusting it
HEADING_MIN_DIST  = 0.3   # minimum baseline displacement (m) to trust GPS heading
# The absolute heading is CONFIRMED once, from the first stretch of forward
# travel (a deliberate ~1 m nudge in real-time, or the first straight leg in
# replay), then held via gyro integration. Re-deriving it from every short GPS
# baseline is what made the displayed heading jitter / flip — so we don't.
HEADING_ESTABLISH_DIST = 1.0   # m of travel needed to confirm the GPS heading
HEADING_START_YAW = math.pi / 2.0  # assume the robot starts facing North (0=E, π/2=N)
HEADING_MAX_AGE_S = 1.5   # is_heading_valid() rejects a heading older than this —
                          # a stale fused_yaw (e.g. right after a turn or a pause)
                          # can be several degrees off, which at typical landmark
                          # detection range turns into metres of lateral scatter

# GPS antenna lever arm relative to the LiDAR/camera reference frame (the frame
# camera_extrinsics / lidar_extrinsics are defined in), in robot body
# coordinates (x=forward, y=left). The antenna sits 20cm behind, 20cm right,
# 10cm below that point, so correcting TO that point moves forward+left.
# Uncorrected, this ~28cm horizontal offset gets rotated into a different ENU
# direction every time the heading changes, scattering repeat sightings of the
# same landmark instead of letting them land on top of each other.
GPS_ANTENNA_OFFSET_BODY = (0.20, 0.20)   # (dx_forward, dy_left), metres

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


def _angle_diff(a: float, b: float) -> float:
    """Signed shortest-path difference (a − b) wrapped to (−π, π]."""
    d = (a - b) % (2 * math.pi)
    return d - 2 * math.pi if d > math.pi else d


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
        self._gps_heading_t: float | None = None  # msg-stamp time _gps_heading was last set
        self._latest_gps_t: float | None = None    # msg-stamp time of the most recent GPS fix
        # /gps/fix's NavSatStatus alone cannot tell RTK Fixed (cm-accurate) apart
        # from RTK Float / DGPS (can be 0.1-several metres off) — both report the
        # same STATUS_GBAS_FIX value. /gps/rtk_status carries the real distinction.
        self._rtk_fixed: bool = False
        self._current_x: float | None = None
        self._current_y: float | None = None
        self._gps_window: deque = deque()  # (t, x, y) baseline samples for heading

        self._gps_track: list[dict] = []
        self._track_dirty = False

        # Gyro-integrated yaw — confirmed once from GPS travel, then gyro-held.
        self._fused_yaw: float = 0.0
        self._fused_yaw_init: bool = False
        self._last_imu_t: float | None = None
        self._last_gyro_z: float = 0.0   # latest yaw rate for straight-motion detection
        # Heading confirmation: start assuming North, lock to the GPS course once
        # the robot has driven HEADING_ESTABLISH_DIST forward.
        self._heading_established: bool = False
        self._establish_origin: tuple | None = None   # (x, y) where the baseline starts

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
        gps_topic       = topics.get('gps',        '/gps/fix')
        imu_topic       = topics.get('imu',        '/xsens/imu/data')
        pose_out_topic  = topics.get('gps_pose',   '/gps_pose')
        rtk_status_topic = topics.get('gps_status', '/gps/rtk_status')

        self._pub = self.create_publisher(PoseStamped, pose_out_topic, 10)
        self.create_subscription(NavSatFix, gps_topic, self._on_gps, BEST_EFFORT)
        self.create_subscription(Imu,       imu_topic, self._on_imu, BEST_EFFORT)
        self.create_subscription(String, rtk_status_topic, self._on_rtk_status, BEST_EFFORT)

        # Periodically flush GPS track to disk
        self.create_timer(30.0, self._flush_track)

        self.get_logger().info(f'GpsLocalizer ready → {pose_out_topic}')
        if self._origin_lat is not None:
            self.get_logger().info(
                f'Origin loaded: {self._origin_lat:.7f}, {self._origin_lon:.7f}'
            )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_rtk_status(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self._rtk_fixed = bool(d.get('rtk_fixed', False))

    def _on_imu(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.orientation
        with self._lock:
            self._current_yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
            self._last_gyro_z = msg.angular_velocity.z
            if not self._fused_yaw_init:
                # Start assuming North; the first ~1 m of travel confirms the real
                # heading (see _on_gps). Gyro integrates turns from here.
                self._fused_yaw = HEADING_START_YAW
                self._fused_yaw_init = True
            else:
                dt = t - self._last_imu_t if self._last_imu_t is not None else 0.0
                # Integrate gyro yaw rate — valid for any motion, especially in-place rotation
                if 0.0 < dt < 0.5:
                    self._fused_yaw += msg.angular_velocity.z * dt
            self._last_imu_t = t

    def _on_gps(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        lat, lon = msg.latitude, msg.longitude
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._current_lat = lat
            self._current_lon = lon
            self._latest_gps_t = t
            if self._origin_lat is None:
                self._origin_lat = lat
                self._origin_lon = lon
                self._save_origin()
                self.get_logger().info(f'GPS origin auto-set: {lat:.7f}, {lon:.7f}')
            x_ant, y_ant = gps_to_enu(lat, lon, self._origin_lat, self._origin_lon)

            # Lever-arm correction: shift from the antenna's GPS fix to the
            # LiDAR/camera reference point, using the best heading estimate
            # available so far (one sample stale at most — negligible).
            heading_now = self._fused_yaw if self._fused_yaw_init else (
                self._gps_heading if self._gps_heading is not None else self._current_yaw
            )
            ofx, ofy = GPS_ANTENNA_OFFSET_BODY
            cos_h, sin_h = math.cos(heading_now), math.sin(heading_now)
            x = x_ant + ofx * cos_h - ofy * sin_h
            y = y_ant + ofx * sin_h + ofy * cos_h

            # ── Heading: confirm forward direction once, then maintain ────────
            # Until confirmed, heading = North assumption + gyro. Once the robot has
            # moved HEADING_ESTABLISH_DIST, the straight-line course over that
            # baseline IS the heading — this also locks which way is "forward".
            if self._establish_origin is None:
                self._establish_origin = (x, y)

            if not self._heading_established:
                ox, oy = self._establish_origin
                if math.hypot(x - ox, y - oy) >= HEADING_ESTABLISH_DIST:
                    course = math.atan2(y - oy, x - ox)
                    self._fused_yaw           = course
                    self._gps_heading         = course
                    self._gps_heading_t       = t
                    self._heading_established = True
                    self.get_logger().info(
                        f'[heading] confirmed from {HEADING_ESTABLISH_DIST:.0f} m '
                        f'travel: {math.degrees(course):.0f}°  (was assuming North)')
            else:
                # Maintenance after confirmation:
                #  • straight driving  → snap heading to the GPS course, which WIPES
                #    any accumulated gyro drift (gyro is exact for rate, drifts over time).
                #  • in-place rotation → GPS gives no course, so the gyro integration
                #    in _on_imu carries the heading (the fallback).
                #  • the forward/reverse 180° choice is LOCKED against the held
                #    heading, so backing up never flips the displayed heading.
                self._gps_window.append((t, x, y))
                while self._gps_window and t - self._gps_window[0][0] > HEADING_WINDOW_S:
                    self._gps_window.popleft()
                if self._gps_window:
                    t0, x0, y0 = self._gps_window[0]
                    dx, dy = x - x0, y - y0
                    if t - t0 >= HEADING_MIN_DT and math.hypot(dx, dy) >= HEADING_MIN_DIST:
                        gps_yaw = math.atan2(dy, dx)
                        # Lock the 180° ambiguity: if the travel course opposes the
                        # held heading, we're reversing → flip back to body-forward.
                        if abs(_angle_diff(gps_yaw, self._fused_yaw)) > math.pi / 2:
                            gps_yaw = _angle_diff(gps_yaw + math.pi, 0.0)
                        self._gps_heading   = gps_yaw
                        self._gps_heading_t = t
                        # Going straight (not rotating in place) → adopt GPS, clearing drift.
                        if abs(self._last_gyro_z) < 0.05:   # < ~3 deg/s
                            self._fused_yaw = gps_yaw
            self._current_x = x
            self._current_y = y
            yaw = self._fused_yaw if self._fused_yaw_init else (
                self._gps_heading if self._gps_heading is not None else self._current_yaw
            )
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
            yaw = self._fused_yaw if self._fused_yaw_init else (
                self._gps_heading if self._gps_heading is not None else self._current_yaw
            )
            return self._current_lat, self._current_lon, yaw

    def get_current_enu(self) -> tuple | None:
        """Returns (x, y, yaw) in local ENU or None."""
        with self._lock:
            if self._current_x is None:
                return None
            yaw = self._fused_yaw if self._fused_yaw_init else (
                self._gps_heading if self._gps_heading is not None else self._current_yaw
            )
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
        """True only while the pose is currently trustworthy for landmark placement.

        Two independent failure modes, both checked here:
        - Staleness: _fused_yaw can drift (gyro integration) between GPS
          corrections, especially right after a turn or a pause, by enough to
          turn a sharp landmark fix into metres of lateral error. Requiring a
          *recent* GPS heading update is a cheap proxy for "fused_yaw is
          currently trustworthy", since straight-line driving (the common
          case while passing a row of landmarks) keeps refreshing it.
        - RTK quality: /gps/fix's NavSatStatus can't tell RTK Fixed (~cm) apart
          from RTK Float / DGPS (0.1-several metres) — both report the same
          STATUS_GBAS_FIX value. Without this, a degraded fix silently feeds
          metre-scale position error into every landmark seen during it.
        """
        with self._lock:
            if not self._rtk_fixed:
                return False
            if self._gps_heading is None or self._gps_heading_t is None:
                return False
            if self._latest_gps_t is None:
                return False
            return (self._latest_gps_t - self._gps_heading_t) < HEADING_MAX_AGE_S

    def reset_for_replay(self):
        """Clear in-memory state for a fresh replay session (keeps persisted origin file)."""
        with self._lock:
            track_snapshot    = list(self._gps_track)
            should_archive    = self._track_dirty   # only archive if new GPS data arrived
            self._gps_track   = []
            self._track_dirty = False
            self._gps_heading    = None
            self._gps_heading_t  = None
            self._latest_gps_t   = None
            self._current_x      = None
            self._current_y      = None
            self._current_lat    = None
            self._current_lon    = None
            self._gps_window.clear()
            self._fused_yaw_init = False
            self._heading_established = False
            self._establish_origin    = None
            self._last_imu_t     = None
            self._last_gyro_z    = 0.0
            self._rtk_fixed      = False
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
