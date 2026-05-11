import shutil
import threading
import time
import logging

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState, PointCloud2, Image, Imu, NavSatFix

log = logging.getLogger(__name__)

SENSOR_TIMEOUT = 2.0

BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class TelemetryNode(Node):
    def __init__(self, data_dir: str, topics: dict):
        super().__init__("telemetry_node")
        self._data_dir = data_dir
        self._lock = threading.Lock()
        self._pose    = [0.0] * 7
        self._battery = -1.0
        self._last_ts: dict[str, float] = {}

        t = topics
        self.create_subscription(PoseStamped,    t["pose"],      self._cb_pose,            BEST_EFFORT)
        self.create_subscription(BatteryState,   t["battery"],   self._cb_battery,         10)
        self.create_subscription(PointCloud2,    t["lidar"],     self._heartbeat("lidar"),  BEST_EFFORT)
        self.create_subscription(Image,          t["zed_front"], self._heartbeat("zed_front"), BEST_EFFORT)
        self.create_subscription(Image,          t["zed_back"],  self._heartbeat("zed_back"),  BEST_EFFORT)
        self.create_subscription(Imu,            t["imu"],       self._heartbeat("imu"),    BEST_EFFORT)
        self.create_subscription(NavSatFix,      t["gps"],       self._heartbeat("gps"),    BEST_EFFORT)

        for key in ("lidar", "zed_front", "zed_back", "imu", "gps"):
            self._last_ts[key] = 0.0

    def _cb_pose(self, msg: PoseStamped):
        p, q = msg.pose.position, msg.pose.orientation
        with self._lock:
            self._pose = [
                round(p.x, 4), round(p.y, 4), round(p.z, 4),
                round(q.x, 4), round(q.y, 4), round(q.z, 4), round(q.w, 4),
            ]

    def _cb_battery(self, msg: BatteryState):
        with self._lock:
            self._battery = round(float(msg.percentage) * 100, 1)

    def _heartbeat(self, key: str):
        def cb(_msg):
            self._last_ts[key] = time.time()
        return cb

    def snapshot(self) -> dict:
        now = time.time()
        with self._lock:
            pose = list(self._pose)
            batt = self._battery
        sensors = {k: (now - ts < SENSOR_TIMEOUT) for k, ts in self._last_ts.items()}
        try:
            total, used, _ = shutil.disk_usage(self._data_dir)
            storage_pct = round(used / total * 100, 1)
        except Exception:
            storage_pct = -1.0
        return {
            "t":           round(now, 2),
            "pose":        pose,
            "batt":        batt,
            "sensors":     sensors,
            "storage_pct": storage_pct,
        }