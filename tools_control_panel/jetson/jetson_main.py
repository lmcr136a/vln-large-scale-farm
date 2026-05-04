import argparse
import logging
import os
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
from sensor_msgs.msg import PointCloud2

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


def load_config(path: str) -> dict:
    with open(os.path.expanduser(path)) as f:
        return yaml.safe_load(f)


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
    parser.add_argument("--config", default="~/farm_config.yaml")
    args = parser.parse_args()
    cfg_path = os.path.expanduser(args.config)

    cfg = load_config(cfg_path)
    rclpy.init()

    base_node   = rclpy.create_node(cfg["ros2"]["node_name"])
    cmd_vel_pub = base_node.create_publisher(Twist, cfg["ros2"]["cmd_vel_topic"], 10)

    telemetry = TelemetryNode(
        data_dir=os.path.expanduser(cfg["paths"]["data_dir"]),
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

    log.info(f"Jetson agent running (config: {cfg_path})")
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        log.info("Shutting down")
        radio.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()