"""
Local PC radio bridge.
- Serial: receives telemetry/events from Jetson, sends commands to Jetson.
- WebSocket client: connects to Lab PC remote_server (/bridge namespace) via Tailscale.
  Forwards everything bidirectionally.
Run: python radio_bridge.py [--config farm_config.yaml]
"""
import argparse
import json
import logging
import serial
import sys
import threading
import time
import yaml

import socketio  # pip install "python-socketio[client]"

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("radio_bridge")

MAX_BYTES = 1900


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


class RadioBridge:
    def __init__(self, cfg):
        self._cfg = cfg
        self._ser = None
        self._sio = socketio.Client(reconnection=True, reconnection_delay=3)
        self._tx_lock = threading.Lock()
        self._running = False

    # ── Serial ────────────────────────────────────────────────────────────────

    def _open_serial(self):
        r = self._cfg["radio"]
        self._ser = serial.Serial(
            port=r["serial_port"], baudrate=r["baud_rate"],
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, rtscts=True, timeout=1,
        )
        log.info(f"Serial opened: {r['serial_port']}")

    def _serial_rx_loop(self):
        while self._running:
            try:
                line = self._ser.readline()
                if not line:
                    continue
                msg = json.loads(line.decode().strip())
                # Forward everything Jetson sends to Lab PC
                self._sio.emit("from_robot", msg, namespace="/bridge")
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
            except Exception as e:
                log.error(f"Serial TX: {e}")

    # ── SocketIO to Lab PC ────────────────────────────────────────────────────

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
            # Lab PC -> radio -> Jetson
            self._send_to_robot(data)

    def run(self):
        self._running = True
        self._open_serial()
        self._setup_sio()

        lab_url = self._cfg["internet"]["lab_ws_url"].replace("ws://", "http://").replace("wss://", "https://")
        # Connect in background (will retry automatically)
        threading.Thread(
            target=lambda: self._sio.connect(lab_url, namespaces=["/bridge"]),
            daemon=True,
        ).start()

        # Serial RX blocks here
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
    parser.add_argument("--config", default="farm_config.yaml")
    args = parser.parse_args()
    cfg = load_config(args.config)
    RadioBridge(cfg).run()


if __name__ == "__main__":
    main()
