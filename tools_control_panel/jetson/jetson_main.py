import argparse
import base64
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

import math
import numpy as np

def _euler_xyz_to_matrix(rpy_deg):
    r, p, y = [math.radians(a) for a in rpy_deg]
    Rx = np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    Ry = np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    Rz = np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
    return Rz @ Ry @ Rx

def _quat_xyzw_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])

def _matrix_to_quat_xyzw(R):
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        return [((R[2,1]-R[1,2])*s), ((R[0,2]-R[2,0])*s),
                ((R[1,0]-R[0,1])*s), (0.25/s)]
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * math.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])
        return [0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s]
    elif R[1,1] > R[2,2]:
        s = 2.0 * math.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])
        return [(R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s]
    else:
        s = 2.0 * math.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])
        return [(R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s]
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import String as _StdString

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from telemetry_node import TelemetryNode
from radio_comm import RadioComm
from internet_comm import InternetComm
from commander import Commander
from station_uploader import StationUploader
from autonomous.autonomous_mode import AutonomousController
from gps.gps_localizer import GpsLocalizer
from perception.landmark_store import LandmarkStore
from perception.landmark_detector import LandmarkDetector
from perception.landmark_map import GpsMapLoop
from perception.scene_describer import SceneDescriber
from sensor.safety_guard import SafetyGuard

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
log = logging.getLogger("jetson_main")

TELEMETRY_HZ = 2.0

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1,
)


def _scale_jpeg_b64(b64: str, size: int, long_edge: bool = False) -> str | None:
    """Scale a JPEG base64 string using OpenCV (always on Jetson).
    size = target width, or (when long_edge=True) the target for the longer side
    so the result fits within size px regardless of orientation."""
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
        if long_edge:
            scale = size / max(h, w)
            new_w = max(1, round(w * scale))
            new_h = max(1, round(h * scale))
        else:
            new_w = size
            new_h = max(1, round(h * size / w))
        small = _cv2.resize(img, (new_w, new_h), interpolation=_cv2.INTER_AREA)
        ok, buf = _cv2.imencode('.jpg', small, [_cv2.IMWRITE_JPEG_QUALITY, 50])
        if not ok:
            log.warning("_scale_jpeg_b64: imencode failed")
            return None
        return _b64.b64encode(buf.tobytes()).decode()
    except Exception as e:
        log.warning(f"_scale_jpeg_b64: {e}")
        return None


class RecorderProxy:
    _ANN_TTL = 3.0  # seconds to show a YOLO annotation before reverting to raw feed

    def __init__(self, internet, telemetry=None, radio=None, image_width=60,
                 manual_check=None, image_interval=1.5, radio_camera="front",
                 landmark_detector=None, scene_describer=None,
                 panel_width=200, panel_hz=3.0):
        self._internet           = internet
        self._telemetry          = telemetry
        self._radio              = radio
        self._last_radio         = 0.0
        self._radio_width        = image_width
        self._panel_width        = panel_width   # long-edge px for control-panel RGB
        self._panel_hz           = panel_hz      # control-panel send rate (own throttle)
        self._last_panel: dict[str, float] = {}  # camera -> last panel send time
        self._manual_check       = manual_check
        self._image_int          = image_interval
        self._radio_cam          = radio_camera
        self._landmark_detector  = landmark_detector
        self._scene_describer    = scene_describer
        self._yolo_ann: dict[str, tuple[float, str]] = {}  # camera -> (ts, b64)

    def set_yolo_annotation(self, camera: str, b64: str):
        """Cache a YOLO-annotated frame. Injected into the stream until TTL expires."""
        self._yolo_ann[camera] = (time.time(), b64)

    def emit(self, event: str, data=None, namespace=None):
        if not (isinstance(event, str) and event.endswith("_frame")) or not data:
            return
        camera = event[:-len("_frame")]   # "back_frame" -> "back"
        raw_b64 = data.get("data", "")
        if not raw_b64:
            return

        # Panel feed has its OWN sparse rate (panel_hz), independent of the live
        # internet QUALITY tier. Gate FIRST (cheap) so surplus frames are dropped
        # before the expensive decode/resize/encode — this is what keeps the feed
        # from backing up at 2x/3x replay.
        now = time.time()
        if self._panel_hz > 0 and \
           now - self._last_panel.get(camera, 0.0) >= 1.0 / self._panel_hz:
            self._last_panel[camera] = now
            # Send YOLO annotation while fresh, otherwise raw frame
            ann = self._yolo_ann.get(camera)
            send_b64 = ann[1] if ann and (now - ann[0]) < self._ANN_TTL else raw_b64

            # Shrink to a small long-edge so each frame is tiny → low latency.
            if self._panel_width:
                send_b64 = _scale_jpeg_b64(send_b64, self._panel_width, long_edge=True) or send_b64

            self._internet.emit_rgb_b64(send_b64, camera, data.get("t"))
        if self._telemetry:
            self._telemetry.touch(f"zed_{camera}")
        # YOLO/VLM are disabled — skip their per-frame callbacks entirely so a
        # replay never wastes a b64-decode on data nothing consumes.
        if self._landmark_detector and self._landmark_detector.is_enabled():
            self._landmark_detector.on_image_b64(camera, raw_b64)  # always raw, no feedback loop
        if self._scene_describer:
            self._scene_describer.on_image_b64(camera, raw_b64)
        if camera == self._radio_cam and self._radio:
            if self._manual_check and self._manual_check():
                return
            if time.time() - self._last_radio < self._image_int:
                return
            small = _scale_jpeg_b64(raw_b64, self._radio_width)
            if small and self._radio.send_bulk({
                "type":   "radio_frame",
                "camera": self._radio_cam,
                "data":   small,
            }):
                self._last_radio = time.time()

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

    _DEFAULTS keys map 1-to-1 with what the web/radio clients display as
    process_status.  Each entry has:
      check  — substring passed to `pgrep -f`; must be specific enough to
                avoid false-positive matches against unrelated processes.
      cmd    — command sent to tmux to (re)start the process.
      window — tmux window name  (mutually exclusive with pane)
      pane   — tmux pane target  (e.g. "Sensors.0")
    """

    _DEFAULTS = {
        # key              check pattern (pgrep -f)       restart cmd                                                              window/pane
        "jetson_agent": {"check": "jetson_main.py",        "window": "Main"},
        "slam":         {"check": "glim_rosnode",           "cmd": "bash launch_files/launch_slam.sh",                              "window": "SLAM"},
        "map_saver":    {"check": "save_map_glim",          "cmd": "python3 tools_control_panel/mapping/save_map_glim_groundseg.py","window": "2Dmap"},
        "ground":       {"check": "launch_ground_seg.sh",   "cmd": "bash launch_files/launch_ground_seg.sh",                       "window": "Ground"},
        "gps":          {"check": "rtk_gps_node.py",        "cmd": "python3 scripts/rtk_gps_node.py",                              "pane":   "Sensors.2"},
        "lidar":        {"check": "launch_robosense.sh",    "cmd": "bash launch_files/launch_robosense.sh",                        "pane":   "Sensors.0"},
        "imu":          {"check": "launch_xsens.sh",        "cmd": "bash launch_files/launch_xsens.sh",                            "pane":   "Sensors.1"},
    }

    # Human-readable labels forwarded to the UI (radio + internet)
    _LABELS = {
        "jetson_agent": "Jetson Main",
        "slam":         "GLIM SLAM",
        "map_saver":    "2D Map Saver",
        "ground":       "Ground Seg",
        "gps":          "RTK GPS",
        "lidar":        "LiDAR",
        "imu":          "IMU",
    }

    _JETSON_RESTART_CMD = "bash launch_files/control_panel_jetson.sh"
    _RESTART_DELAY_S    = 10
    _POLL_INTERVAL_S    = 5

    def __init__(self, session: str, cfg_windows: dict | None = None):
        self._session = session
        self._windows = dict(self._DEFAULTS)
        if cfg_windows:
            for k, v in cfg_windows.items():
                self._windows.setdefault(k, {}).update(v)
        # status: {key: {"alive": bool, "pid": int|None, "label": str}}
        self._status: dict = {
            k: {"alive": None, "pid": None, "label": self._LABELS.get(k, k)}
            for k in self._windows
        }
        self._lock = threading.Lock()
        threading.Thread(target=self._poll_loop, daemon=True).start()

    def get_status(self) -> dict:
        """Return a shallow copy of the current process status dict."""
        with self._lock:
            return {k: dict(v) for k, v in self._status.items()}

    def get_status_flat(self) -> dict:
        """
        Return a compact dict suitable for embedding in telemetry:
          {"slam": true, "ground": false, "lidar": true, ...}
        None means 'unknown / poll not yet completed'.
        """
        with self._lock:
            return {k: v["alive"] for k, v in self._status.items()}

    def restart(self, key: str) -> bool:
        spec = self._windows.get(key)
        if not spec or "cmd" not in spec:
            log.warning(f"TmuxMonitor.restart: no restart cmd for '{key}'")
            return False
        target = spec.get("pane") or spec.get("window")
        if not target:
            return False
        t = f"{self._session}:{target}"
        try:
            for _ in range(3):
                subprocess.run(["tmux", "send-keys", "-t", t, "C-c", ""], timeout=3)
                time.sleep(1.0)
            time.sleep(1.0)
            subprocess.run(["tmux", "send-keys", "-t", t, spec["cmd"], "Enter"], timeout=3)
            log.info(f"TmuxMonitor: restarted '{key}' in {t}")
            return True
        except Exception as e:
            log.error(f"TmuxMonitor restart '{key}': {e}")
            return False

    def restart_jetson_main(self) -> bool:
        t = f"{self._session}:Main"
        restart_cmd = f"sleep 5 && {self._JETSON_RESTART_CMD}"
        try:
            subprocess.run(["tmux", "send-keys", "-t", t, restart_cmd, "Enter"], timeout=3)
            time.sleep(0.3)
            subprocess.run(["pkill", "-f", "control_panel_jetson.sh"], timeout=3)
            return True
        except Exception as e:
            log.error(f"restart_jetson_main: {e}")
            return False

    def _poll_loop(self):
        while True:
            new_status = {}
            for key, spec in self._windows.items():
                check = spec.get("check", "")
                label = self._LABELS.get(key, key)
                alive = None
                pid   = None
                if check:
                    try:
                        r = subprocess.run(
                            ["pgrep", "-f", check],
                            capture_output=True, timeout=2,
                        )
                        alive = r.returncode == 0
                        if alive and r.stdout.strip():
                            try:
                                pid = int(r.stdout.strip().split()[0])
                            except (ValueError, IndexError):
                                pass
                    except Exception:
                        alive = None
                new_status[key] = {"alive": alive, "pid": pid, "label": label}

            with self._lock:
                prev = dict(self._status)
                self._status = new_status

            # log only on alive↔dead transitions, not every poll cycle
            for key, s in new_status.items():
                was = prev.get(key, {}).get("alive")
                if s["alive"] is False and was is True:
                    log.warning(f"TmuxMonitor: '{key}' ({s['label']}) went down")
                elif s["alive"] is True and was is False:
                    log.info(f"TmuxMonitor: '{key}' ({s['label']}) is back up")

            time.sleep(self._POLL_INTERVAL_S)


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
    R    = _euler_xyz_to_matrix(rpy)
    T         = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = t
    return T


def _peek_bag_start_ns(bag_path: str) -> int | None:
    """Earliest message timestamp (epoch ns) in a rosbag, without playing it.

    Same epoch as ZED's TIME_REFERENCE.IMAGE timestamps (verified directly
    against this recorder's output) — lets replay align SVO frame 0 with the
    bag's actual first message instead of guessing a fixed startup delay.
    """
    import glob
    import sqlite3
    db_files = glob.glob(os.path.join(bag_path, '*.db3'))
    if not db_files:
        return None
    try:
        con = sqlite3.connect(db_files[0])
        row = con.execute('SELECT MIN(timestamp) FROM messages').fetchone()
        con.close()
        return int(row[0]) if row and row[0] is not None else None
    except Exception as e:
        log.warning(f'_peek_bag_start_ns: {e}')
        return None


def apply_extrinsic(T_B_L: np.ndarray, pos: list, quat_xyzw: list):
    R_M_L        = _quat_xyzw_to_matrix(quat_xyzw)
    T_M_L        = np.eye(4)
    T_M_L[:3,:3] = R_M_L
    T_M_L[:3, 3] = pos
    R_B_L = T_B_L[:3, :3]
    t_B_L = T_B_L[:3,  3]
    T_L_B        = np.eye(4)
    T_L_B[:3,:3] = R_B_L.T
    T_L_B[:3, 3] = -R_B_L.T @ t_B_L
    T_M_B  = T_M_L @ T_L_B
    pos_c  = T_M_B[:3, 3].tolist()
    quat_c = _matrix_to_quat_xyzw(T_M_B[:3,:3])
    return pos_c, quat_c


def main():
    parser = argparse.ArgumentParser()
    default_cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/farm_config.yaml")
    parser.add_argument("--config", default=default_cfg)
    parser.add_argument("--replay", metavar="SESSION_DIR", default=None,
                        help="Replay a recorded session: plays rosbag + SVO2 files together")
    parser.add_argument("--replay-speed", type=float, default=1.0,
                        help="Replay playback speed multiplier (e.g. 3.0 = 3x faster)")
    args = parser.parse_args()
    cfg_path     = os.path.abspath(args.config)
    replay_dir   = os.path.abspath(args.replay) if args.replay else None
    replay_speed = max(args.replay_speed, 0.01)

    cfg = load_config(cfg_path)

    # CYCLONEDDS_URI must be set before any DDS library is loaded — setting it
    # here inside Python is too late if other nodes already started discovery.
    # The correct place is control_panel_jetson.sh (see cyclonedds_jetson.xml).
    # We still warn loudly if it is missing so the issue is easy to spot.
    if not os.environ.get("CYCLONEDDS_URI"):
        log.warning(
            "CYCLONEDDS_URI not set — ddsi UDP write errors may appear. "
            "Add 'export CYCLONEDDS_URI=file:///path/to/cyclonedds_jetson.xml' "
            "to control_panel_jetson.sh before launching any ROS2 process."
        )

    rclpy.init()

    base_node   = rclpy.create_node(cfg["ros2"]["node_name"])
    cmd_vel_pub = base_node.create_publisher(Twist, cfg["ros2"]["cmd_vel_topic"], 10)

    telemetry = TelemetryNode(
        data_dir=cfg["paths"]["data_dir"],
        topics=cfg["ros2"]["topics"],
        cameras=[c.get("name") for c in cfg.get("recording", {}).get("zed_cameras", [])
                 if c.get("name")],
        float_acc_limit=cfg.get("gps", {}).get("float_accuracy_limit_m", 0.10),
    )
    uploader  = StationUploader(cfg_path)
    radio     = RadioComm(port=cfg["radio"]["serial_port"], baud=cfg["radio"]["baud_rate"],
                          bulk_threshold=cfg["radio"].get("bulk_tx_threshold", 64))
    internet  = InternetComm(lab_url=cfg["internet"]["lab_ws_url"])

    # GPS localizer — replaces SLAM pose
    gps_localizer = GpsLocalizer(cfg)

    # Landmark store + detector
    _proj_dir   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    _data_dir   = os.path.normpath(os.path.join(_proj_dir, cfg["paths"]["data_dir"]))
    _map_dir    = os.path.normpath(os.path.join(_proj_dir, cfg["paths"]["map_dir"]))
    landmark_store    = LandmarkStore(os.path.join(_data_dir, "landmarks.json"))
    landmark_detector = LandmarkDetector(gps_localizer, landmark_store, cfg)
    # YOLO landmark detection disabled — no inference, no annotated-frame stream.
    # Keeping the object (disabled) so the wiring below stays no-op without guards:
    # start() returns early, is_enabled() is False (no LiDAR sub), and the image/
    # depth/pointcloud callbacks just store data nothing consumes.
    landmark_detector._enabled = False
    landmark_store.set_origin(*gps_localizer.get_origin())  # may be None if no origin yet

    # GPS top-down map loop
    _crop_file = os.path.join(os.path.dirname(cfg_path), 'crop_field.json')  # config/, like mission.json
    gps_map_loop = GpsMapLoop(
        gps_localizer, landmark_store, internet, _map_dir,
        interval=cfg.get("map", {}).get("update_interval", 3.0),
        crop_file=_crop_file,
    )

    proxy     = SocketIOProxy(radio, internet, uploader, telemetry)

    # Scene description (VLM) disabled — the control panel streams the camera
    # images only, with no per-camera VLM text. Leaving this off also frees the
    # GPU the ZED cameras need to open reliably.
    scene_describer = None

    tmux_session     = cfg.get("tmux", {}).get("session", "vln")
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
        topics = cfg["recording"].get("rosbag_topics") or (
            list(cfg["ros2"]["topics"].values()) + [cfg["ros2"]["cmd_vel_topic"]])
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
            except Exception:
                pass
        if _rosbag_proc[0]:
            _rosbag_proc[0].terminate()
            _rosbag_proc[0] = None

    auto_ctrl = AutonomousController(cmd_vel_pub, proxy, cfg,
                                      start_recording=start_rec_cb,
                                      stop_recording=stop_rec_cb,
                                      heading_check=gps_localizer.is_heading_established,
                                      heading_reset=gps_localizer.reset_heading,
                                      rtk_check=gps_localizer.is_drivable)
    commander = Commander(cmd_vel_pub, auto_ctrl, cfg_path)
    commander.set_flip_heading(gps_localizer.flip_heading)   # panel N/S heading flip

    _restart_lock = threading.Lock()

    # Default network-recovery command — overridable in farm_config.yaml under
    # network.reconnect_cmd. Runs on the Jetson when the panel sends a
    # 'network_reconnect' command (which can travel over radio even when wifi /
    # tailscale is down, since that's the whole point of the radio link).
    _net_reconnect_cmd = cfg.get("network", {}).get(
        "reconnect_cmd",
        "nmcli radio wifi off; sleep 3; nmcli radio wifi on; sleep 5; "
        "nmcli device connect wlan0; sudo systemctl restart tailscaled",
    )

    def _run_shell(cmd_str: str):
        cmd_str = (cmd_str or "").strip()
        if not cmd_str:
            return
        log.warning(f"Running shell command from panel over link: {cmd_str}")
        try:
            subprocess.Popen(cmd_str, shell=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            log.error(f"shell command failed: {e}")

    def on_command(cmd):
        ctype = cmd.get("cmd")
        if ctype == "start_recording":
            start_rec_cb(cmd.get("dirname", "manual"))
        elif ctype == "stop_recording":
            stop_rec_cb()
        elif ctype == "shell":
            # Free-form recovery command from the control panel (e.g. bring the
            # network back). Sent over radio so it works even with no internet.
            _run_shell(cmd.get("command", ""))
        elif ctype == "network_reconnect":
            _run_shell(_net_reconnect_cmd)
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

    pc_topic    = cfg["internet"]["pointcloud_topic"]
    pc_interval = float(cfg["internet"]["pointcloud_interval"])
    _latest_pc: list = [None]

    def pc_callback(msg: PointCloud2):
        _latest_pc[0] = pc2_to_numpy(msg)

    base_node.create_subscription(PointCloud2, pc_topic, pc_callback, BEST_EFFORT_QOS)

    _safety_status: list    = [{}]
    _safety_last_t: list    = [0.0]
    _SAFETY_TIMEOUT         =  1.5

    def safety_cb(msg: _StdString):
        try:
            _safety_status[0] = json.loads(msg.data)
            _safety_last_t[0] = time.time()
        except Exception:
            pass

    base_node.create_subscription(_StdString, '/safety_checker', safety_cb, 10)

    def _safety_status_getter():
        return _safety_status[0] if time.time() - _safety_last_t[0] < _SAFETY_TIMEOUT else {}

    def _safety_notify(msg):
        proxy.emit('robot_status', {'status': msg}, namespace='/')

    _safety_cfg     = cfg.get('safety', {})
    _safety_wait    = float(_safety_cfg.get('wait_sec', 10.0))
    _recover_speed  = float(_safety_cfg.get('recover_speed') or cfg['autonomous']['t1'][0])
    safety_guard = SafetyGuard(_safety_status_getter, commander, _recover_speed,
                               wait_sec=_safety_wait, notify=_safety_notify,
                               auto_ctrl=auto_ctrl)
    commander.set_safety_guard(safety_guard)   # panel toggle → guard.set_enabled()
    safety_guard.start()

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
        # Seed with the current time so a stale map_latest.png left on disk from a
        # previous session is NOT pushed on startup (that caused the old SLAM map
        # to flash). Only maps GLIM regenerates this session (newer mtime) are sent.
        last_mtime    = time.time()
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

    _pose_send_state = {"last": 0.0, "radio": 0.0}
    POSE_INTERNET_HZ = 5.0
    POSE_RADIO_HZ    = float(cfg.get("radio", {}).get("pose_hz", 5.0))

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

        raw_yaw = _math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
        )
        now_s = time.time()
        if internet.connected and (now_s - _pose_send_state["last"]) >= 1.0 / POSE_INTERNET_HZ:
            _pose_send_state["last"] = now_s
            internet.send_pose(p.x, p.y, raw_yaw)
            gps_map_loop.update_robot_pose({"x": p.x, "y": p.y, "yaw": raw_yaw})
        if POSE_RADIO_HZ > 0 and (now_s - _pose_send_state["radio"]) >= 1.0 / POSE_RADIO_HZ:
            _pose_send_state["radio"] = now_s
            radio.send({"type": "pose", "x": p.x, "y": p.y, "yaw": raw_yaw})

    raw_pose_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST, depth=1,
    )
    base_node.create_subscription(_PS, raw_pose_topic, pose_corrector_cb, raw_pose_qos)

    # GPS pose → internet / radio / map (separate from SLAM corrector above)
    gps_pose_topic = cfg["ros2"]["topics"].get("gps_pose", "/gps_pose")

    def gps_pose_cb(msg: _PS):
        p, q = msg.pose.position, msg.pose.orientation
        yaw = _math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
        )
        heading_valid = gps_localizer.is_heading_valid()
        now_s = time.time()
        # Pose's own sim-time (bag stamp) so the panel can pair it with the RGB
        # frame of the same instant during replay.
        pose_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if internet.connected and (now_s - _pose_send_state["last"]) >= 1.0 / POSE_INTERNET_HZ:
            _pose_send_state["last"] = now_s
            internet.send_pose(p.x, p.y, yaw, t=pose_t)
            gps_map_loop.update_robot_pose(
                {"x": p.x, "y": p.y, "yaw": yaw, "heading_valid": heading_valid}
            )
        if POSE_RADIO_HZ > 0 and (now_s - _pose_send_state["radio"]) >= 1.0 / POSE_RADIO_HZ:
            _pose_send_state["radio"] = now_s
            radio.send({"type": "pose", "x": p.x, "y": p.y, "yaw": yaw})

    base_node.create_subscription(_PS, gps_pose_topic, gps_pose_cb, raw_pose_qos)

    threading.Thread(target=map_watch_loop, daemon=True).start()

    recorder   = None
    svo_player = None
    radio_img_w = cfg.get("radio", {}).get("image_width", 60)
    rec_proxy = RecorderProxy(internet, telemetry, radio, image_width=radio_img_w,
                              manual_check=commander.is_manual,
                              image_interval=cfg.get("radio", {}).get("image_interval", 1.5),
                              radio_camera=cfg.get("radio", {}).get("camera", "front"),
                              landmark_detector=landmark_detector,
                              scene_describer=scene_describer,
                              panel_width=cfg.get("control_panel", {}).get("rgb_width", 200),
                              panel_hz=cfg.get("control_panel", {}).get("rgb_hz", 3.0))

    # YOLO annotated frames → cached in rec_proxy, injected into the live stream for TTL seconds
    def _on_annotated_frame(camera: str, jpeg_bytes: bytes):
        b64 = base64.b64encode(jpeg_bytes).decode()
        rec_proxy.set_yolo_annotation(camera, b64)

    landmark_detector.set_annotated_frame_callback(_on_annotated_frame)

    # Load/generate the GPS map before opening the RGBD (ZED) cameras so the
    # web panel has a map to show as soon as camera streaming starts.
    gps_map_loop.start()

    if replay_dir:
        # ── Replay mode: play rosbag + SVO2 files together ──────────────────
        # Clear in-memory state so old track/heading don't bleed into this session
        gps_localizer.reset_for_replay()

        def _on_svo_frame(camera: str, jpeg_bytes: bytes, ts_ns: int = None):
            b64 = base64.b64encode(jpeg_bytes).decode()
            data = {"data": b64}
            if ts_ns is not None:
                data["t"] = ts_ns / 1e9   # sim-time seconds, for panel RGB↔GPS sync
            rec_proxy.emit(f"{camera}_frame", data)

        def _on_svo_depth(camera: str, depth_arr, intrinsics: dict):
            # YOLO/landmark detection is disabled — depth would only be stored for
            # an inference step that never runs, so skip it during replay.
            if landmark_detector.is_enabled():
                landmark_detector.on_depth_frame(camera, depth_arr, intrinsics)

        from sensor.svo_player import SvoPlayer
        svo_player = SvoPlayer(
            session_dir=replay_dir,
            on_frame=_on_svo_frame,
            # No depth consumer when landmark detection is off → SvoPlayer skips
            # depth compute entirely (DEPTH_MODE.NONE), which is the big replay win.
            on_depth=(_on_svo_depth if landmark_detector.is_enabled() else None),
            stream_fps=cfg["recording"].get("web_stream_hz", 5),
            speed=replay_speed,
        )
        svo_player.start()
        log.info(f"Replay mode: {replay_dir} (speed={replay_speed}x) — waiting for first SVO frame...")

        bag_path = os.path.join(replay_dir, 'rosbag')
        bag_start_ns = _peek_bag_start_ns(bag_path)

        # Align bag playback to the SVO stream's own recorded start time, instead
        # of a guessed fixed delay. ZED's recorded timestamps and the rosbag's
        # recorded timestamps share the same epoch (recorder.py starts both
        # close together), so the gap between them is meaningful and small.
        # Once the bag starts, SvoPlayer's /clock subscription takes over and
        # keeps every subsequent frame in lockstep for the rest of the session.
        got_frame = svo_player.ready_event.wait(timeout=60.0)
        if got_frame and bag_start_ns is not None:
            gap_s = (bag_start_ns - svo_player.first_frame_ts_ns) / 1e9
            log.info(f"Replay sync: bag's first message is {gap_s:+.3f}s "
                     f"(recorded time) from the first SVO frame")
            if gap_s > 0:
                time.sleep(gap_s / replay_speed)
        else:
            log.warning("Replay sync: no SVO frame / bag timestamp available — "
                        "starting bag play after a fixed 2s delay instead")
            time.sleep(2.0)

        if not os.path.exists(bag_path):
            log.error(f"Replay: rosbag not found at {bag_path}")
        else:
            log.info(f"Replay: starting ros2 bag play {bag_path} --rate {replay_speed}")
            import subprocess as _sp
            _sp.Popen(['ros2', 'bag', 'play', bag_path, '--clock', '--rate', str(replay_speed)])
    else:
        # ── Live mode: open ZED cameras ───────────────────────────────────────
        try:
            from sensor.recorder import MultiSensorRecorder, resolve_zed_serials
            rec_cfg  = cfg["recording"]
            recorder = MultiSensorRecorder(base_node, output_base_dir=cfg["paths"]["data_dir"])
            for cam in resolve_zed_serials(rec_cfg.get("zed_cameras", [])):
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

    # Feed LiDAR to landmark detector
    def _lidar_to_detector(msg: PointCloud2):
        pts = pc2_to_numpy(msg)
        landmark_detector.on_pointcloud(pts)

    if landmark_detector.is_enabled():
        lidar_topic = cfg["ros2"]["topics"].get("lidar", "/rslidar_points")
        base_node.create_subscription(PointCloud2, lidar_topic,
                                      _lidar_to_detector, BEST_EFFORT_QOS)

    landmark_detector.start()

    def _push_landmarks():
        """Send landmark data + ENU coordinates to web panel."""
        lms = landmark_store.get_all()
        origin_lat, origin_lon = gps_localizer.get_origin()
        if origin_lat is None:
            return
        from gps.gps_localizer import gps_to_enu
        for lm in lms:
            x, y = gps_to_enu(lm['lat'], lm['lon'], origin_lat, origin_lon)
            lm['enu_x'] = round(x, 2)
            lm['enu_y'] = round(y, 2)
        internet.send_event('landmarks_updated', {'landmarks': lms})

    def _landmark_push_loop():
        while True:
            time.sleep(10.0)
            if internet.connected:
                try:
                    _push_landmarks()
                except Exception as e:
                    log.warning(f"landmark push: {e}")

    threading.Thread(target=_landmark_push_loop, daemon=True).start()

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
            snap["safety_status"]    = (
                _safety_status[0]
                if time.time() - _safety_last_t[0] < _SAFETY_TIMEOUT
                else {}
            )

            if tmux_monitor:
                # process_status: {"slam": true, "ground": false, ...}
                # process_status_detail: {"slam": {"alive": true, "pid": 1234, "label": "GLIM SLAM"}, ...}
                snap["process_status"]        = tmux_monitor.get_status_flat()
                snap["process_status_detail"] = tmux_monitor.get_status()

            radio.send({"type": "telemetry", "data": snap})
            internet.send_telemetry(snap)
            time.sleep(interval)

    threading.Thread(target=telemetry_loop, daemon=True).start()

    # Keep GPS map waypoints in sync with autonomous controller
    _orig_auto_start = auto_ctrl.start
    def _auto_start_with_map(waypoints, *args, **kwargs):
        gps_map_loop.update_waypoints(waypoints)
        return _orig_auto_start(waypoints, *args, **kwargs)
    auto_ctrl.start = _auto_start_with_map

    executor = MultiThreadedExecutor()
    executor.add_node(telemetry)
    executor.add_node(auto_ctrl)
    executor.add_node(gps_localizer)
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
    try:
        shutdown.wait()
    except OSError:
        pass

    log.info("Shutting down")
    stop_rec_cb()
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