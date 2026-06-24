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

SENSOR_TIMEOUT = 5.0
# /scout_status is published at 50 Hz. If the base CAN link drops (the
# "Failed to send CAN frame" condition), the feedback stops too, so a short
# silence is a reliable "base not responding" signal. Generous enough to ride
# out a couple of missed frames without false alarms.
BASE_COMM_TIMEOUT = 1.5

# scout_msgs is only on the path when the scout workspace is sourced (it is, in
# control_panel_jetson.sh). Guard the import so the agent still runs elsewhere
# (e.g. replay on a dev box) — base monitoring just stays disabled there.
try:
    from scout_msgs.msg import ScoutStatus
    _HAVE_SCOUT_MSGS = True
except Exception:
    ScoutStatus = None
    _HAVE_SCOUT_MSGS = False

BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class TelemetryNode(Node):
    def __init__(self, data_dir: str, topics: dict, cameras: list | None = None):
        super().__init__("telemetry_node")
        self._data_dir = data_dir
        # Camera names expected to be live for the system to count as "ready"
        # (e.g. ["front", "back", "left", "right"]). Each streams a frame →
        # touch("zed_<name>"), so its liveness is tracked in _last_ts.
        self._cameras = list(cameras) if cameras else []
        self._lock = threading.Lock()
        self._pose    = [0.0] * 7
        self._battery = -1.0
        self._last_ts: dict[str, float] = {}
        self._gps_status: dict = {}

        # Scout base health (CAN link). _base_last_t = last /scout_status receipt;
        # if it goes stale the base isn't responding (CAN frames failing to send).
        self._base_last_t:   float = 0.0
        self._base_error:    int   = 0
        self._base_vehicle:  int   = 0

        t = topics
        self.create_subscription(PoseStamped,  t["pose"],    self._cb_pose,          BEST_EFFORT)
        self.create_subscription(BatteryState, t["battery"], self._cb_battery,       10)
        self.create_subscription(PointCloud2,  t["lidar"],   self._heartbeat("lidar"),   BEST_EFFORT)
        self.create_subscription(Imu,          t["imu"],     self._heartbeat("imu"),     BEST_EFFORT)
        if "gps" in t:
            self.create_subscription(NavSatFix, t["gps"],   self._heartbeat("gps"),     BEST_EFFORT)
        if "gps_status" in t:
            self.create_subscription(String, t["gps_status"], self._cb_gps_status, 10)
        if _HAVE_SCOUT_MSGS:
            self.create_subscription(ScoutStatus, "/scout_status", self._cb_scout, BEST_EFFORT)

        for key in ("lidar", "imu", "gps"):
            self._last_ts[key] = 0.0
        # Seed every expected camera (plus front/back as a sane default) so they
        # show ✗ until their first frame, instead of being silently absent.
        for name in (self._cameras or ["front", "back"]):
            self._last_ts[f"zed_{name}"] = 0.0

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

    def _cb_scout(self, msg):
        with self._lock:
            self._base_last_t  = time.time()
            self._base_error   = int(msg.error_code)
            self._base_vehicle = int(msg.vehicle_state)

    def touch(self, key: str):
        self._last_ts[key] = time.time()

    def _net_monitor_loop(self):
        INTERVAL = 3.0
        # Interfaces to skip (loopback, docker, virtual)
        # Only track WiFi interfaces (wlan*, wlp*, wlP* etc.)
        WIFI_PREFIX = ("wl",)
        while True:
            time.sleep(INTERVAL)
            try:
                counters = psutil.net_io_counters(pernic=True)
                now = time.time()
                new_stats = {}
                with self._net_lock:
                    prev = dict(self._net_prev)
                for iface, c in counters.items():
                    if not any(iface.startswith(p) for p in WIFI_PREFIX):
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
            base_last_t = self._base_last_t
            base_error  = self._base_error
            base_vehicle = self._base_vehicle
        # base_status.comm == False ⇒ no /scout_status lately ⇒ CAN link down
        # (the "Failed to send CAN frame" condition). error_code surfaces other
        # base faults (under-voltage, motor fault, RC lost, ...).
        base_status = {
            "comm":          _HAVE_SCOUT_MSGS and (now - base_last_t < BASE_COMM_TIMEOUT),
            "error_code":    base_error,
            "vehicle_state": base_vehicle,
            "monitored":     _HAVE_SCOUT_MSGS,
        }
        sensors = {k: (now - ts < SENSOR_TIMEOUT) for k, ts in self._last_ts.items()}

        # ── system_ready: every health condition met (internet NOT required) ──
        #   • all expected cameras streaming
        #   • lidar + imu + gps alive
        #   • RTK Float or Fixed
        #   • Scout base CAN link up
        # The panel shows "Ready"/"OK" off this; it deliberately omits internet so
        # a link drop never clears the badge (telemetry just stops arriving; the
        # panel keeps the last value).
        cams = self._cameras or ["front", "back"]
        cam_ok  = all(sensors.get(f"zed_{c}", False) for c in cams)
        core_ok = (sensors.get("lidar", False) and sensors.get("imu", False)
                   and sensors.get("gps", False))
        rtk_ok  = bool(gps_status.get("rtk_fixed") or gps_status.get("rtk_float"))
        base_ok = bool(base_status.get("comm"))
        system_ready = bool(cam_ok and core_ok and rtk_ok and base_ok)

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
            "base_status": base_status,
            "system_ready": system_ready,
        }