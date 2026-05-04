"""
Jetson agent main.
Starts: ROS2 telemetry node, radio comm, internet comm, commander, scheduler.
"""
import logging
import os
import sys
import threading
import time
import yaml

import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from telemetry_node import TelemetryNode
from radio_comm import RadioComm
from internet_comm import InternetComm
from commander import Commander
from scheduler import Scheduler

# Reuse existing autonomous controller from tools_control_panel
from autonomous.autonomous_mode import AutonomousController

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
log = logging.getLogger("jetson_main")

CFG_PATH = os.path.expanduser("~/farm_config.yaml")
TELEMETRY_HZ = 2.0  # radio telemetry send rate


class SocketIOProxy:
    """
    Drop-in for flask_socketio.SocketIO used by AutonomousController.
    Redirects emit() calls to radio + internet instead of a browser.
    """

    def __init__(self, radio: RadioComm, internet: InternetComm):
        self._radio = radio
        self._internet = internet

    def emit(self, event: str, data=None, namespace=None):
        payload = {"type": "event", "event": event, "data": data or {}}
        self._radio.send(payload)
        self._internet.send_event(event, data)


def load_config():
    with open(CFG_PATH) as f:
        return yaml.safe_load(f)


def main():
    cfg = load_config()
    rclpy.init()

    # ROS2 publisher (shared with autonomous controller)
    base_node = rclpy.create_node("jetson_agent_base")
    cmd_vel_pub = base_node.create_publisher(Twist, cfg["ros2"]["cmd_vel_topic"], 10)

    # Telemetry collector
    telemetry = TelemetryNode(os.path.expanduser(cfg["paths"]["data_dir"]))

    # Radio comm — start before commander so on_command is ready
    radio = RadioComm(
        port=cfg["radio"]["serial_port"],
        baud=cfg["radio"]["baud_rate"],
    )
    internet = InternetComm(lab_url=cfg["internet"]["lab_ws_url"])

    proxy = SocketIOProxy(radio, internet)

    auto_ctrl = AutonomousController(cmd_vel_pub, proxy, cfg)

    commander = Commander(cmd_vel_pub, auto_ctrl, CFG_PATH)

    def on_command(cmd: dict):
        commander.handle(cmd)

    radio._on_command = on_command
    internet._on_command = on_command

    radio.start()
    internet.start()

    # Scheduler fires autonomous runs at configured times
    def schedule_start(waypoints):
        auto_ctrl.start(waypoints)

    scheduler = Scheduler(CFG_PATH, schedule_start)
    scheduler.run()

    # Telemetry dispatch loop
    def telemetry_loop():
        interval = 1.0 / TELEMETRY_HZ
        while True:
            snap = telemetry.snapshot()
            snap["mode"] = (
                "auto" if auto_ctrl.is_active()
                else "manual" if commander.is_manual()
                else "idle"
            )
            snap["estop"] = commander.is_estopped()
            radio.send({"type": "telemetry", "data": snap})
            internet.send_telemetry(snap)
            time.sleep(interval)

    threading.Thread(target=telemetry_loop, daemon=True).start()

    # ROS2 spin
    executor = MultiThreadedExecutor()
    executor.add_node(telemetry)
    executor.add_node(auto_ctrl)
    executor.add_node(base_node)

    log.info("Jetson agent running")
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
