import shutil
import threading
import time
import logging

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState, PointCloud2, Image, Imu

log = logging.getLogger(__name__)

SENSOR_TIMEOUT = 2.0  # seconds before marking sensor offline

BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class TelemetryNode(Node):
    """
    Subscribes to ROS2 topics and exposes snapshot() for telemetry dispatch.
    Sensor health is determined by topic heartbeat (SENSOR_TIMEOUT).
    """

    def __init__(self, data_dir: str):
        super().__init__("telemetry_node")
        self._data_dir = data_dir
        self._lock = threading.Lock()

        self._pose = [0.0] * 7  # x y z qx qy qz qw
        self._battery = -1.0
        self._last_ts: dict[str, float] = {
            "lidar": 0.0, "zed_front": 0.0, "zed_back": 0.0, "imu": 0.0
        }

        self.create_subscription(
            PoseStamped, "/glim_ros/localized_curr_pose", self._cb_pose, BEST_EFFORT
        )
        self.create_subscription(
            BatteryState, "/battery_state", self._cb_battery, 10
        )
        self.create_subscription(
            PointCloud2, "/livox/lidar", self._heartbeat("lidar"), BEST_EFFORT
        )
        self.create_subscription(
            Image, "/zed_front/rgb/image_raw", self._heartbeat("zed_front"), BEST_EFFORT
        )
        self.create_subscription(
            Image, "/zed_back/rgb/image_raw", self._heartbeat("zed_back"), BEST_EFFORT
        )
        self.create_subscription(
            Imu, "/imu/data", self._heartbeat("imu"), BEST_EFFORT
        )

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
            "t": round(now, 2),
            "pose": pose,
            "batt": batt,
            "sensors": sensors,
            "storage_pct": storage_pct,
        }
