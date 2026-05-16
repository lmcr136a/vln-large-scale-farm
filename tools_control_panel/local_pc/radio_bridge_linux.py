"""
Local PC radio bridge — Linux version.
Serial port: /dev/ttyUSB* or /dev/ttyACM* (set in farm_config.yaml → radio.serial_port)
Run: python3 radio_bridge_linux.py [--config ../config/farm_config.yaml]
"""
import argparse
import json
import logging
import os
import threading
import time

import serial
import socketio
import yaml

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("radio_bridge")

MAX_BYTES = 1900

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CFG       = os.path.join(_SCRIPT_DIR, "../config/farm_config.yaml")
_DEFAULT_CFG_LOCAL = os.path.join(_SCRIPT_DIR, "../config/local_config.yaml")


def _deep_merge(base: dict, override: dict) -> dict:
    result = base.copy()
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(result.get(k), dict):
            result[k] = _deep_merge(result[k], v)
        else:
            result[k] = v
    return result


def load_config(base_path, local_path=None):
    with open(base_path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    if local_path and os.path.exists(local_path):
        with open(local_path, encoding='utf-8') as f:
            local = yaml.safe_load(f)
        cfg = _deep_merge(cfg, local or {})
        log.info(f"Local config applied: {local_path}")
    return cfg


class RadioBridge:
    def __init__(self, cfg):
        self._cfg     = cfg
        self._ser     = None
        self._sio     = socketio.Client(reconnection=True, reconnection_delay=3)
        self._tx_lock = threading.Lock()
        self._running = False
        self._stats   = {
            "rx_total":    0,
            "tx_total":    0,
            "rx_window":   0,
            "tx_window":   0,
            "last_rx":     {},
            "last_pose":   None,
        }
        self._status_lines = 0

    def _open_serial(self):
        r = self._cfg["radio"]
        self._ser = serial.Serial(
            port=r["serial_port"], baudrate=r["baud_rate"],
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            rtscts=True,
            timeout=1,
            write_timeout=1.0,
        )
        log.info(f"Serial opened: {r['serial_port']}")

    def _serial_rx_loop(self):
        while self._running:
            try:
                line = self._ser.readline()
                if not line:
                    continue
                msg = json.loads(line.decode('utf-8', errors='ignore').strip())
                self._sio.emit("from_robot", msg, namespace="/bridge")
                self._stats["rx_total"]  += 1
                self._stats["rx_window"] += 1
                mtype = msg.get("type", "unknown")
                data  = msg.get("data", msg)
                if isinstance(data, dict):
                    self._stats["last_rx"][mtype] = list(data.keys())
                    if mtype == "telemetry":
                        if "pose" in data:
                            self._stats["last_pose"] = data["pose"]
                        if "sensors" in data:
                            self._stats["last_sensors"] = data["sensors"]
                else:
                    self._stats["last_rx"][mtype] = str(data)[:60]
            except json.JSONDecodeError:
                pass
            except Exception as e:
                if self._running:
                    log.error(f"Serial RX: {e}")
                    time.sleep(0.2)

    def _send_to_robot(self, payload: dict):
        data = (json.dumps(payload, separators=(',', ':')) + '\n').encode()
        if len(data) > MAX_BYTES:
            log.warning(f"Command too large ({len(data)}B), dropping")
            return
        with self._tx_lock:
            try:
                self._ser.write(data)
                self._stats["tx_total"]  += 1
                self._stats["tx_window"] += 1
            except serial.SerialTimeoutException:
                log.error("Serial TX: Write timeout")
            except Exception as e:
                log.error(f"Serial TX: {e}")

    def _setup_sio(self):
        sio = self._sio

        @sio.on("connect", namespace="/bridge")
        def on_connect():
            log.info("Connected to Lab PC")

        @sio.on("disconnect", namespace="/bridge")
        def on_disconnect():
            log.warning("Disconnected from Lab PC")

        @sio.on("to_robot", namespace="/bridge")
        def on_command(data):
            self._send_to_robot(data)

    def _status_loop(self):
        interval = 2.0
        while self._running:
            time.sleep(interval)
            s = self._stats
            rx_hz = s["rx_window"] / interval
            tx_hz = s["tx_window"] / interval
            s["rx_window"] = 0
            s["tx_window"] = 0

            pose = s["last_pose"]
            if pose and isinstance(pose, dict):
                px = pose.get("x", pose.get("position", {}).get("x", "?"))
                py = pose.get("y", pose.get("position", {}).get("y", "?"))
                yaw = pose.get("yaw", pose.get("z", "?"))
                pose_str = f"x={px:.3f}  y={py:.3f}  yaw={yaw:.3f}" if all(
                    isinstance(v, (int, float)) for v in [px, py, yaw]
                ) else str(pose)
            else:
                pose_str = "—"

            sensors = s["last_sensors"]
            if sensors:
                sensor_str = "  ".join(
                    f"{k}:{'✓' if v else '✗'}" for k, v in sensors.items()
                )
            else:
                sensor_str = "—"

            lines = [
                "┌─ Bridge Status ───────────────────────────────────────────────┐",
                f"│  Local PC → Lab PC   {rx_hz:.1f} pkt/s  (total {s['rx_total']})",
                f"│  Lab PC → Local PC   {tx_hz:.1f} pkt/s  (total {s['tx_total']})",
                f"│  Pose     {pose_str}",
                f"│  Sensors  {sensor_str}",
                "└───────────────────────────────────────────────────────────────┘",
            ]

            # move cursor up to overwrite previous output
            if self._status_lines:
                print(f"\033[{self._status_lines}A", end="")
            for l in lines:
                print(f"\033[2K{l}")
            self._status_lines = len(lines)

    def run(self):
        self._running = True
        self._open_serial()
        self._setup_sio()

        lab_url = self._cfg["internet"]["lab_ws_url"] \
            .replace("ws://", "http://").replace("wss://", "https://")
        threading.Thread(
            target=lambda: self._sio.connect(lab_url, namespaces=["/bridge"]),
            daemon=True,
        ).start()
        threading.Thread(target=self._status_loop, daemon=True).start()

        try:
            self._serial_rx_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self._running = False
            self._ser.close()
            self._sio.disconnect()
            log.info("Bridge stopped")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=_DEFAULT_CFG)
    parser.add_argument("--local-config", default=_DEFAULT_CFG_LOCAL)
    args = parser.parse_args()
    cfg = load_config(args.config, args.local_config)
    RadioBridge(cfg).run()


if __name__ == "__main__":
    main()