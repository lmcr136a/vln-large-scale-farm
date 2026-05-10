import argparse
import json
import logging
import os
import signal
import struct
import sys
import threading
import time
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


class RecorderProxy:
    """Captures front_frame/back_frame emits from recorder and sends via internet."""
    def __init__(self, internet):
        self._internet = internet

    def emit(self, event: str, data=None, namespace=None):
        if event in ("front_frame", "back_frame") and data:
            camera = "front" if event == "front_frame" else "back"
            b64 = data.get("data", "")
            if b64:
                self._internet.send_rgb_b64(b64, camera)

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
    """
    Returns 4×4 homogeneous transform T_B_L:
    transforms a point expressed in LiDAR frame into base_link frame.
    Reads lidar_extrinsics.translation ([x,y,z] m) and
               lidar_extrinsics.rotation  ([roll,pitch,yaw] deg) from config.
    """
    ext  = cfg.get("lidar_extrinsics", {})
    t    = np.array(ext.get("translation", [0.0, 0.0, 0.0]), dtype=float)
    rpy  = ext.get("rotation", [0.0, 0.0, 0.0])
    R    = _Rotation.from_euler("xyz", rpy, degrees=True).as_matrix()
    T         = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = t
    return T


def apply_extrinsic(T_B_L: np.ndarray, pos: list, quat_xyzw: list):
    """
    Correct a GLIM-published LiDAR pose to the robot base_link pose.

    GLIM gives T_M_L (LiDAR in map frame).
    We want  T_M_B  (base_link in map frame).

      T_M_B = T_M_L  ×  T_L_B
      T_L_B = inv(T_B_L)

    pos       : [x, y, z]          — LiDAR origin in map frame
    quat_xyzw : [qx, qy, qz, qw]  — LiDAR orientation in map frame
    Returns   : (pos_corrected, quat_corrected_xyzw)
    """
    # T_M_L
    R_M_L       = _Rotation.from_quat(quat_xyzw).as_matrix()
    T_M_L       = np.eye(4)
    T_M_L[:3,:3] = R_M_L
    T_M_L[:3, 3] = pos

    # T_L_B = inv(T_B_L)
    R_B_L = T_B_L[:3, :3]
    t_B_L = T_B_L[:3,  3]
    T_L_B       = np.eye(4)
    T_L_B[:3,:3] = R_B_L.T
    T_L_B[:3, 3] = -R_B_L.T @ t_B_L

    # T_M_B
    T_M_B = T_M_L @ T_L_B
    pos_c  = T_M_B[:3, 3].tolist()
    quat_c = _Rotation.from_matrix(T_M_B[:3,:3]).as_quat().tolist()  # [x,y,z,w]
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
    auto_ctrl = AutonomousController(cmd_vel_pub, proxy, cfg)
    commander = Commander(cmd_vel_pub, auto_ctrl, cfg_path)

    def on_command(cmd):
        commander.handle(cmd)

    radio._on_command    = on_command
    internet._on_command = on_command

    radio.start()
    internet.start()

    Scheduler(cfg_path, auto_ctrl.start).run()

    pc_topic    = cfg["internet"]["pointcloud_topic"]
    pc_interval = float(cfg["internet"]["pointcloud_interval"])
    _latest_pc: list = [None]

    def pc_callback(msg: PointCloud2):
        _latest_pc[0] = pc2_to_numpy(msg)

    base_node.create_subscription(PointCloud2, pc_topic, pc_callback, BEST_EFFORT_QOS)
    log.info(f"Pointcloud source: {pc_topic}")

    def pointcloud_loop():
        while True:
            time.sleep(pc_interval)
            pts = _latest_pc[0]
            if pts is not None and internet.connected:
                internet.send_pointcloud(pts)

    threading.Thread(target=pointcloud_loop, daemon=True).start()

    # Map direct-push loop: sends map_latest.png via internet when quality == "high"
    shutdown = threading.Event()

    def map_watch_loop():
        # Read map_dir from raw yaml — load_config resolves the relative path against
        # the config dir, giving tools_control_panel/output_glim instead of root/output_glim.
        # save_map_glim.py uses raw yaml.safe_load + __file__ anchor, so we match it.
        with open(cfg_path) as _f:
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
        log.info(f"Map watch: {png_path}")
        while not shutdown.is_set():
            if shutdown.wait(timeout=interval):
                break
            if not internet.connected or internet.quality == "low":
                continue
            try:
                mtime = os.path.getmtime(png_path)
                if map_not_found:
                    log.info("Map watch: map found, starting to send")
                    map_not_found = False
                if mtime <= last_mtime:
                    continue
                last_mtime = mtime
                with open(state_path) as f:
                    meta = json.load(f)
                internet.send_map(png_path, meta)
                log.info(f"Map sent ({int(os.path.getsize(png_path)/1024)}KB)")
            except FileNotFoundError:
                if not map_not_found:
                    log.info("Map watch: no map yet, will send once available")
                    map_not_found = True
            except Exception as e:
                log.error(f"Map watch: {e}")

    # ── Pose corrector ────────────────────────────────────────────────────────
    # Subscribes to raw GLIM pose, applies lidar_extrinsics, republishes on
    # cfg["ros2"]["topics"]["pose"] so ALL downstream code (TelemetryNode,
    # AutonomousController, internet sender) automatically gets the correct pose.
    from geometry_msgs.msg import PoseStamped as _PS
    import math as _math

    T_B_L = build_extrinsic_T_B_L(cfg)
    log.info(f"Lidar extrinsics — t={cfg.get('lidar_extrinsics',{}).get('translation')} "
             f"rpy_deg={cfg.get('lidar_extrinsics',{}).get('rotation')}")

    raw_pose_topic = cfg["ros2"]["topics"].get(
        "raw_pose", "/glim_ros/localized_curr_pose")
    corrected_pose_topic = cfg["ros2"]["topics"].get(
        "pose", "/corrected_pose")

    corrected_pose_pub = base_node.create_publisher(
        _PS, corrected_pose_topic, 10)

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

        # Internet push: send raw GLIM yaw so lab PC applies the extrinsic once.
        # Position correction is negligible (zero translation in typical setup).
        if internet.connected:
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
    base_node.create_subscription(
        _PS, raw_pose_topic, pose_corrector_cb, raw_pose_qos)
    log.info(f"Pose corrector: {raw_pose_topic} → {corrected_pose_topic}")

    threading.Thread(target=map_watch_loop, daemon=True).start()

    # ZED cameras via recorder
    recorder = None
    try:
        from sensor.recorder import MultiSensorRecorder
        rec_cfg  = cfg["recording"]
        recorder = MultiSensorRecorder(base_node, output_base_dir=cfg["paths"]["data_dir"])
        rec_proxy = RecorderProxy(internet)
        for cam in rec_cfg.get("zed_cameras", []):
            recorder.add_zed_camera(
                serial_number=cam["serial"],
                name=cam["name"],
                socketio=rec_proxy,
                sample_interval_sec=rec_cfg.get("sample_interval_sec", 300),
                always_stream=True,
                stream_fps=rec_cfg.get("web_stream_hz", 5),
            )
        log.info(f"ZED cameras initialized: {[c['name'] for c in rec_cfg.get('zed_cameras', [])]}")
    except Exception as e:
        log.warning(f"ZED recorder init failed: {e}")

    def telemetry_loop():
        interval = 1.0 / TELEMETRY_HZ
        while True:
            snap = telemetry.snapshot()
            snap["mode"]  = (
                "auto"   if auto_ctrl.is_active() else
                "manual" if commander.is_manual() else
                "idle"
            )
            snap["estop"] = commander.is_estopped()
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
    # Stop recorder threads before ROS2 context is destroyed
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