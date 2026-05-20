import argparse
import json
import logging
import os
import signal
import struct
import subprocess
import sys
import threading
import time
from datetime import datetime
import yaml

import numpy as np
from scipy.spatial.transform import Rotation as _Rotation
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from telemetry_node import TelemetryNode
from radio_comm import RadioComm
from internet_comm import InternetComm
from commander import Commander
from scheduler import Scheduler
from station_uploader import StationUploader
from autonomous.autonomous_mode import AutonomousController

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
log = logging.getLogger("jetson_main")

TELEMETRY_HZ = 2.0

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1,
)


def _scale_jpeg_b64(b64: str, width: int) -> str | None:
    """Scale a JPEG base64 string to given width using OpenCV (always on Jetson)."""
    try:
        import base64 as _b64
        import numpy as _np
        import cv2 as _cv2
        raw  = _b64.b64decode(b64)
        arr  = _np.frombuffer(raw, dtype=_np.uint8)
        img  = _cv2.imdecode(arr, _cv2.IMREAD_COLOR)
        if img is None:
            log.warning("_scale_jpeg_b64: imdecode returned None")
            return None
        h, w = img.shape[:2]
        new_h = max(1, round(h * width / w))
        small = _cv2.resize(img, (width, new_h), interpolation=_cv2.INTER_AREA)
        ok, buf = _cv2.imencode('.jpg', small, [_cv2.IMWRITE_JPEG_QUALITY, 50])
        if not ok:
            log.warning("_scale_jpeg_b64: imencode failed")
            return None
        return _b64.b64encode(buf.tobytes()).decode()
    except Exception as e:
        log.warning(f"_scale_jpeg_b64: {e}")
        return None


class RecorderProxy:
    RADIO_INTERVAL = 3.0   # seconds between radio frames

    def __init__(self, internet, telemetry=None, radio=None, image_width=60):
        self._internet    = internet
        self._telemetry   = telemetry
        self._radio       = radio
        self._last_radio  = 0.0
        self._radio_width = image_width  # read from cfg['radio']['image_width']

    def emit(self, event: str, data=None, namespace=None):
        if event in ("front_frame", "back_frame") and data:
            camera = "front" if event == "front_frame" else "back"
            b64 = data.get("data", "")
            if b64:
                self._internet.send_rgb_b64(b64, camera)
                if self._telemetry:
                    self._telemetry.touch(f"zed_{camera}")
                # Radio: physically-front camera (labeled 'back' in config), rate-limited
                if camera == "back" and self._radio:
                    now = time.time()
                    if now - self._last_radio >= self.RADIO_INTERVAL:
                        self._last_radio = now
                        small = _scale_jpeg_b64(b64, self._radio_width)
                        if small:
                            self._radio.send({
                                "type":   "radio_frame",
                                "camera": "back",
                                "data":   small,
                            })

    def start_background_task(self, *a, **kw): pass


class SocketIOProxy:
    def __init__(self, radio, internet, uploader, telemetry):
        self._radio     = radio
        self._internet  = internet
        self._uploader  = uploader
        self._telemetry = telemetry

    def emit(self, event: str, data=None, namespace=None):
        payload = {"type": "event", "event": event, "data": data or {}}
        self._radio.send(payload)
        self._internet.send_event(event, data)
        if event == "auto_mode_completed":
            pose = self._telemetry.snapshot()["pose"]
            self._uploader.notify_run_completed(pose)


from config_loader import load_config


class TmuxMonitor:
    """
    Polls tmux windows for process liveness and can restart them via send-keys.
    Self-restart of jetson_main.py stays in window Main (index 2).
    """

    _DEFAULTS = {
        "jetson_agent": {"check": "jetson_main.py", "window": "Main"},
        "slam":      {"check": "glim_rosnode",   "cmd": "bash launch_files/launch_slam.sh",             "window": "SLAM"},
        "map_saver": {"check": "save_map_glim",  "cmd": "python3 tools_control_panel/mapping/save_map_glim.py", "window": "2Dmap"},
        "obstacle":  {"check": "safety_checker", "cmd": "python3 tools_scout_control/safety_checker.py","window": "O.D."},
        "gps":       {"check": "rtk_gps_node",   "cmd": "python3 scripts/rtk_gps_node.py",              "pane":   "Sensors.2"},
        "lidar":     {"check": "robosense",       "cmd": "bash launch_files/launch_robosense.sh",        "pane":   "Sensors.0"},
        "imu":       {"check": "xsens",           "cmd": "bash launch_files/launch_xsens.sh",           "pane":   "Sensors.1"},
    }

    _JETSON_RESTART_CMD = "bash launch_files/control_panel_jetson.sh"
    _RESTART_DELAY_S    = 10

    def __init__(self, session: str, cfg_windows: dict | None = None):
        self._session = session
        self._windows = dict(self._DEFAULTS)
        if cfg_windows:
            for k, v in cfg_windows.items():
                self._windows.setdefault(k, {}).update(v)
        self._status: dict = {}
        self._lock = threading.Lock()
        threading.Thread(target=self._poll_loop, daemon=True).start()

    def get_status(self) -> dict:
        with self._lock:
            return dict(self._status)

    def restart(self, key: str) -> bool:
        spec = self._windows.get(key)
        if not spec:
            return False
        target = spec.get("pane") or spec.get("window")
        if not target:
            return False
        t = f"{self._session}:{target}"
        try:
            for i in range(5):
                subprocess.run(["tmux", "send-keys", "-t", t, "C-c", ""], timeout=3)
                time.sleep(2.0)
            subprocess.run(["tmux", "send-keys", "-t", t, spec["cmd"], "Enter"], timeout=3)
            log.info(f"TmuxMonitor: restarted {key} in {t}")
            return True
        except Exception as e:
            log.error(f"TmuxMonitor restart {key}: {e}")
            for i in range(10):
                subprocess.run(["tmux", "send-keys", "-t", t, "C-c", ""], timeout=3)
                time.sleep(2.0)
            subprocess.run(["tmux", "send-keys", "-t", t, spec["cmd"], "Enter"], timeout=3)
            log.info(f"TmuxMonitor: restarted {key} in {t}")
            return False

    def restart_jetson_main(self) -> bool:
        """
        Restart jetson_main.py in window Main (index 2):
        1. Spawn a background shell that waits 2 s then sends the restart command
           to window Main — by then the old process has died and the shell is free.
        2. Kill the current jetson_main.py — this process terminates here.
        After _RESTART_DELAY_S seconds the new instance starts in window Main.
        """
        t = f"{self._session}:Main"
        restart_cmd = f"sleep {self._RESTART_DELAY_S} && {self._JETSON_RESTART_CMD}"
        try:
            # Background shell sends the command once the pane is free
            subprocess.Popen(
                ["bash", "-c", f'sleep 2 && tmux send-keys -t "{t}" "{restart_cmd}" Enter']
            )
            time.sleep(0.3)
            # Kill current process — exits from here
            subprocess.run(["pkill", "-f", "jetson_main.py"], timeout=3)
            return True
        except Exception as e:
            log.error(f"restart_jetson_main: {e}")
            return False

    def _poll_loop(self):
        while True:
            status = {}
            for key, spec in self._windows.items():
                check = spec.get("check", "")
                try:
                    r = subprocess.run(
                        ["pgrep", "-f", check],
                        capture_output=True, timeout=2
                    )
                    status[key] = r.returncode == 0
                except Exception:
                    status[key] = None
            with self._lock:
                self._status = status
            time.sleep(5)


def pc2_to_numpy(msg: PointCloud2) -> np.ndarray:
    offsets = {f.name: f.offset for f in msg.fields}
    ox, oy, oz = offsets.get("x", 0), offsets.get("y", 4), offsets.get("z", 8)
    step = msg.point_step
    raw  = bytes(msg.data)
    n    = msg.width * msg.height
    pts  = np.empty((n, 3), dtype=np.float32)
    for i in range(n):
        b = i * step
        pts[i, 0] = struct.unpack_from("f", raw, b + ox)[0]
        pts[i, 1] = struct.unpack_from("f", raw, b + oy)[0]
        pts[i, 2] = struct.unpack_from("f", raw, b + oz)[0]
    return pts


def build_extrinsic_T_B_L(cfg: dict) -> np.ndarray:
    ext  = cfg.get("lidar_extrinsics", {})
    t    = np.array(ext.get("translation", [0.0, 0.0, 0.0]), dtype=float)
    rpy  = ext.get("rotation", [0.0, 0.0, 0.0])
    R    = _Rotation.from_euler("xyz", rpy, degrees=True).as_matrix()
    T         = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = t
    return T


def apply_extrinsic(T_B_L: np.ndarray, pos: list, quat_xyzw: list):
    R_M_L       = _Rotation.from_quat(quat_xyzw).as_matrix()
    T_M_L       = np.eye(4)
    T_M_L[:3,:3] = R_M_L
    T_M_L[:3, 3] = pos
    R_B_L = T_B_L[:3, :3]
    t_B_L = T_B_L[:3,  3]
    T_L_B       = np.eye(4)
    T_L_B[:3,:3] = R_B_L.T
    T_L_B[:3, 3] = -R_B_L.T @ t_B_L
    T_M_B = T_M_L @ T_L_B
    pos_c  = T_M_B[:3, 3].tolist()
    quat_c = _Rotation.from_matrix(T_M_B[:3,:3]).as_quat().tolist()
    return pos_c, quat_c


def main():
    parser = argparse.ArgumentParser()
    default_cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/farm_config.yaml")
    parser.add_argument("--config", default=default_cfg)
    args = parser.parse_args()
    cfg_path = os.path.abspath(args.config)

    cfg = load_config(cfg_path)
    rclpy.init()

    base_node   = rclpy.create_node(cfg["ros2"]["node_name"])
    cmd_vel_pub = base_node.create_publisher(Twist, cfg["ros2"]["cmd_vel_topic"], 10)

    telemetry = TelemetryNode(
        data_dir=cfg["paths"]["data_dir"],
        topics=cfg["ros2"]["topics"],
    )
    uploader  = StationUploader(cfg_path)
    radio     = RadioComm(port=cfg["radio"]["serial_port"], baud=cfg["radio"]["baud_rate"])
    internet  = InternetComm(lab_url=cfg["internet"]["lab_ws_url"])

    proxy     = SocketIOProxy(radio, internet, uploader, telemetry)

    tmux_session    = cfg.get("tmux", {}).get("session", "vln")
    tmux_cfg_windows = cfg.get("tmux", {}).get("windows", None)
    try:
        tmux_monitor = TmuxMonitor(tmux_session, tmux_cfg_windows)
    except Exception as e:
        log.warning(f"TmuxMonitor init failed: {e}")
        tmux_monitor = None

    _rosbag_proc = [None]
    _rec_lock    = threading.Lock()
    _rec_active  = [False]

    def start_rec_cb(subdir: str):
        with _rec_lock:
            if _rec_active[0]:
                log.warning("start_recording ignored: already recording")
                return
            _rec_active[0] = True
        import subprocess, yaml as _yaml
        try:
            with open(cfg_path, encoding='utf-8') as _f:
                _raw = _yaml.safe_load(_f)
            _proj = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            _data = os.path.normpath(os.path.join(_proj, _raw["paths"]["data_dir"]))
            ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
            rec_dir  = os.path.join(_data, subdir, ts)
            bag_path = os.path.join(rec_dir, 'rosbag')
            os.makedirs(rec_dir, exist_ok=True)
        except Exception as e:
            log.error(f"Recording path setup failed: {e}")
            return
        if recorder:
            try:
                recorder.start_recording(output_dir=rec_dir)
            except Exception as e:
                log.error(f"Camera recording start failed: {e}")
        topics = list(cfg["ros2"]["topics"].values()) + [cfg["ros2"]["cmd_vel_topic"]]
        try:
            _rosbag_proc[0] = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', bag_path] + list(set(topics)))
        except Exception as e:
            log.error(f"Rosbag start failed: {e}")

    def stop_rec_cb():
        with _rec_lock:
            _rec_active[0] = False
        if recorder:
            try:
                recorder.stop_recording()
            except Exception as e:
                log.error(f"Camera recording stop failed: {e}")
        if _rosbag_proc[0]:
            _rosbag_proc[0].terminate()
            _rosbag_proc[0] = None

    auto_ctrl = AutonomousController(cmd_vel_pub, proxy, cfg,
                                      start_recording=start_rec_cb,
                                      stop_recording=stop_rec_cb)
    commander = Commander(cmd_vel_pub, auto_ctrl, cfg_path)

    _restart_lock = threading.Lock()  # prevents double-fire from radio+internet

    def on_command(cmd):
        ctype = cmd.get("cmd")
        if ctype == "start_recording":
            start_rec_cb(cmd.get("dirname", "manual"))
        elif ctype == "stop_recording":
            stop_rec_cb()
        elif ctype == "restart_window":
            key = cmd.get("window", "")
            if key == "jetson_agent":
                if tmux_monitor and _restart_lock.acquire(blocking=False):
                    log.info("Self-restart requested via browser")
                    tmux_monitor.restart_jetson_main()
                    time.sleep(5)
            elif tmux_monitor:
                ok = tmux_monitor.restart(key)
                log.info(f"restart_window '{key}': {'ok' if ok else 'failed'}")
        else:
            commander.handle(cmd)

    radio._on_command    = on_command
    internet._on_command = on_command

    radio.start()
    internet.start()

    Scheduler(cfg_path, auto_ctrl.start, lab_url=cfg['internet']['lab_ws_url']).run()

    pc_topic    = cfg["internet"]["pointcloud_topic"]
    pc_interval = float(cfg["internet"]["pointcloud_interval"])
    _latest_pc: list = [None]

    def pc_callback(msg: PointCloud2):
        _latest_pc[0] = pc2_to_numpy(msg)

    base_node.create_subscription(PointCloud2, pc_topic, pc_callback, BEST_EFFORT_QOS)

    def pointcloud_loop():
        while True:
            time.sleep(pc_interval)
            pts = _latest_pc[0]
            if pts is not None and internet.connected:
                internet.send_pointcloud(pts)

    threading.Thread(target=pointcloud_loop, daemon=True).start()

    shutdown = threading.Event()

    def map_watch_loop():
        with open(cfg_path, encoding='utf-8') as _f:
            _raw = yaml.safe_load(_f)
        proj_dir    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        raw_map_dir = os.path.expanduser(_raw["paths"]["map_dir"])
        if not os.path.isabs(raw_map_dir):
            raw_map_dir = os.path.join(proj_dir, raw_map_dir)
        map_dir    = os.path.normpath(raw_map_dir)
        png_path   = os.path.join(map_dir, cfg["paths"].get("map_image",  "map_latest.png"))
        state_path = os.path.join(map_dir, cfg["paths"].get("map_state",  "map_state.json"))
        interval   = cfg.get("map", {}).get("update_interval", 1.0)
        last_mtime    = 0.0
        map_not_found = False
        while not shutdown.is_set():
            if shutdown.wait(timeout=interval):
                break
            if not internet.connected or internet._quality == "low":
                continue
            try:
                mtime = os.path.getmtime(png_path)
                if map_not_found:
                    map_not_found = False
                if mtime <= last_mtime:
                    continue
                last_mtime = mtime
                with open(state_path) as f:
                    meta = json.load(f)
                internet.send_map(png_path, meta)
            except FileNotFoundError:
                if not map_not_found:
                    map_not_found = True
            except Exception as e:
                log.error(f"Map watch: {e}")

    from geometry_msgs.msg import PoseStamped as _PS
    import math as _math

    T_B_L = build_extrinsic_T_B_L(cfg)

    raw_pose_topic       = cfg["ros2"]["topics"].get("raw_pose", "/glim_ros/localized_curr_pose")
    corrected_pose_topic = cfg["ros2"]["topics"].get("pose", "/corrected_pose")
    corrected_pose_pub   = base_node.create_publisher(_PS, corrected_pose_topic, 10)

    _pose_send_state = {"last": 0.0}
    POSE_INTERNET_HZ = 5.0

    def pose_corrector_cb(msg: _PS):
        p, q = msg.pose.position, msg.pose.orientation
        pos_c, q_c = apply_extrinsic(
            T_B_L,
            [p.x, p.y, p.z],
            [q.x, q.y, q.z, q.w],
        )
        out = _PS()
        out.header          = msg.header
        out.header.frame_id = "map"
        out.pose.position.x, out.pose.position.y, out.pose.position.z = pos_c
        out.pose.orientation.x, out.pose.orientation.y, \
            out.pose.orientation.z, out.pose.orientation.w = q_c
        corrected_pose_pub.publish(out)

        now_s = time.time()
        if internet.connected and (now_s - _pose_send_state["last"]) >= 1.0 / POSE_INTERNET_HZ:
            _pose_send_state["last"] = now_s
            raw_yaw = _math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
            )
            internet.send_pose(p.x, p.y, raw_yaw)

    raw_pose_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST, depth=1,
    )
    base_node.create_subscription(_PS, raw_pose_topic, pose_corrector_cb, raw_pose_qos)

    threading.Thread(target=map_watch_loop, daemon=True).start()

    recorder = None
    try:
        from sensor.recorder import MultiSensorRecorder
        rec_cfg  = cfg["recording"]
        recorder = MultiSensorRecorder(base_node, output_base_dir=cfg["paths"]["data_dir"])
        radio_img_w = cfg.get("radio", {}).get("image_width", 60)
        rec_proxy = RecorderProxy(internet, telemetry, radio, image_width=radio_img_w)
        for cam in rec_cfg.get("zed_cameras", []):
            recorder.add_zed_camera(
                serial_number=cam["serial"],
                name=cam["name"],
                socketio=rec_proxy,
                sample_interval_sec=rec_cfg.get("sample_interval_sec", 300),
                always_stream=True,
                stream_fps=rec_cfg.get("web_stream_hz", 5),
            )
    except Exception as e:
        log.warning(f"ZED recorder init failed: {e}")

    _telem_size_last = [0.0]

    def telemetry_loop():
        interval = 1.0 / TELEMETRY_HZ
        while True:
            snap = telemetry.snapshot()
            snap["mode"]  = (
                "auto"   if auto_ctrl.is_active() else
                "manual" if commander.is_manual() else
                "idle"
            )
            snap["estop"]            = commander.is_estopped()
            snap["internet"]         = internet.connected
            snap["radio_quality"]    = radio.get_quality()
            snap["radio_rtt_ms"]     = radio.get_rtt_ms()
            snap["internet_quality"] = internet.get_quality()
            snap["internet_rtt_ms"]  = internet.get_rtt_ms()
            if tmux_monitor:
                snap["tmux_status"] = tmux_monitor.get_status()

            # Periodic size breakdown — helps diagnose bandwidth usage
            now_t = time.time()
            if now_t - _telem_size_last[0] >= 60.0:
                _telem_size_last[0] = now_t
                pkt = json.dumps({"type": "telemetry", "data": snap}, separators=(',', ':'))
                breakdown = sorted(
                    ((k, len(json.dumps(v, separators=(',', ':')).encode()))
                     for k, v in snap.items()),
                    key=lambda x: -x[1],
                )

            radio.send({"type": "telemetry", "data": snap})
            internet.send_telemetry(snap)
            time.sleep(interval)

    threading.Thread(target=telemetry_loop, daemon=True).start()

    executor = MultiThreadedExecutor()
    executor.add_node(telemetry)
    executor.add_node(auto_ctrl)
    executor.add_node(base_node)

    def handle_signal(sig, frame):
        if shutdown.is_set():
            return
        log.info("Shutdown signal received")
        shutdown.set()
        signal.signal(signal.SIGINT,  signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)

    signal.signal(signal.SIGINT,  handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    threading.Thread(target=executor.spin, daemon=True).start()

    log.info(f"Jetson agent running (config: {cfg_path})")
    shutdown.wait()

    log.info("Shutting down")
    stop_rec_cb()  # stop rosbag + camera recording if active
    if recorder:
        try:
            recorder.stop_recording()
            recorder.shutdown()
        except Exception:
            pass
    executor.shutdown(timeout_sec=2.0)
    radio.stop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()