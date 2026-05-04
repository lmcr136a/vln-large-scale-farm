import argparse
import logging
import os
import signal
import struct
import sys
import threading
import time
import yaml

import numpy as np
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
    def __init__(self, internet: "InternetComm"):
        self._internet = internet

    def emit(self, event: str, data=None, namespace=None):
        if event in ("front_frame", "back_frame") and data:
            camera = "front" if event == "front_frame" else "back"
            raw = data.get("data", "")
            if raw:
                import base64
                self._internet.send_rgb(base64.b64decode(raw), camera)

    def start_background_task(self, *a, **kw): pass  # no-op shim
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

    shutdown = threading.Event()

    def handle_signal(sig, frame):
        log.info("Shutdown signal received")
        shutdown.set()

    signal.signal(signal.SIGINT,  handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    threading.Thread(target=executor.spin, daemon=True).start()

    log.info(f"Jetson agent running (config: {cfg_path})")
    shutdown.wait()

    log.info("Shutting down")
    executor.shutdown()
    radio.stop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()