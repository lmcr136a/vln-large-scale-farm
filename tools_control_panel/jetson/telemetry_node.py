import json
import os
import shutil
import threading
import time
import logging

import psutil

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState, PointCloud2, Image, Imu, NavSatFix
from std_msgs.msg import String

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
        self._gps_status: dict = {}

        t = topics
        self.create_subscription(PoseStamped,  t["pose"],    self._cb_pose,          BEST_EFFORT)
        self.create_subscription(BatteryState, t["battery"], self._cb_battery,       10)
        self.create_subscription(PointCloud2,  t["lidar"],   self._heartbeat("lidar"),   BEST_EFFORT)
        self.create_subscription(Imu,          t["imu"],     self._heartbeat("imu"),     BEST_EFFORT)
        if "gps" in t:
            self.create_subscription(NavSatFix, t["gps"],   self._heartbeat("gps"),     BEST_EFFORT)
        if "gps_status" in t:
            self.create_subscription(String, t["gps_status"], self._cb_gps_status, 10)

        for key in ("lidar", "zed_front", "zed_back", "imu", "gps"):
            self._last_ts[key] = 0.0

        # Network traffic tracking: detect hung WiFi (signal shows 100% but no TX)
        self._net_prev: dict = {}   # {iface: (tx_bytes, rx_bytes, timestamp)}
        self._net_stats: dict = {}  # {iface: {"tx_kbps": float, "rx_kbps": float, "alive": bool}}
        self._net_lock = threading.Lock()
        threading.Thread(target=self._net_monitor_loop, daemon=True).start()

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

    def _cb_gps_status(self, msg: String):
        try:
            status = json.loads(msg.data)
            with self._lock:
                self._gps_status = status
        except Exception:
            pass

    def touch(self, key: str):
        self._last_ts[key] = time.time()

    def _net_monitor_loop(self):
        INTERVAL = 3.0
        # Interfaces to skip (loopback, docker, virtual)
        SKIP_PREFIX = ("lo", "docker", "veth", "br-", "virbr")
        while True:
            time.sleep(INTERVAL)
            try:
                counters = psutil.net_io_counters(pernic=True)
                now = time.time()
                new_stats = {}
                with self._net_lock:
                    prev = dict(self._net_prev)
                for iface, c in counters.items():
                    if any(iface.startswith(p) for p in SKIP_PREFIX):
                        continue
                    tx, rx = c.bytes_sent, c.bytes_recv
                    if iface in prev:
                        ptx, prx, pt = prev[iface]
                        dt = max(now - pt, 0.001)
                        tx_kbps = (tx - ptx) / dt / 1024
                        rx_kbps = (rx - prx) / dt / 1024
                        # "alive" = at least some TX in the last interval
                        # (RX alone could just be broadcast noise)
                        alive = tx_kbps > 0.1
                        new_stats[iface] = {
                            "tx_kbps": round(tx_kbps, 1),
                            "rx_kbps": round(rx_kbps, 1),
                            "alive":   alive,
                        }
                    with self._net_lock:
                        self._net_prev[iface] = (tx, rx, now)
                with self._net_lock:
                    self._net_stats = new_stats
            except Exception as e:
                log.warning(f"net_monitor: {e}")

    def _heartbeat(self, key: str):
        def cb(_msg):
            self._last_ts[key] = time.time()
        return cb

    def snapshot(self) -> dict:
        now = time.time()
        with self._lock:
            pose = list(self._pose)
            batt = self._battery
            gps_status = dict(self._gps_status)
        sensors = {k: (now - ts < SENSOR_TIMEOUT) for k, ts in self._last_ts.items()}
        try:
            total, used, _ = shutil.disk_usage(self._data_dir)
            storage_pct = round(used / total * 100, 1)
        except Exception:
            storage_pct = -1.0
        try:
            wifi = os.popen('iwgetid -r').read().strip() or '—'
        except Exception:
            wifi = '—'
        with self._net_lock:
            net_stats = dict(self._net_stats)
        return {
            "t":           round(now, 2),
            "pose":        pose,
            "batt":        batt,
            "sensors":     sensors,
            "storage_pct": storage_pct,
            "wifi":        wifi,
            "gps_status":  gps_status,
            "net":         net_stats,
        }