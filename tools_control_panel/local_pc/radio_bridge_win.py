"""
Local PC radio bridge — Windows version.
Serial port: COM* (set in farm_config.yaml → radio.serial_port)
Run: python radio_bridge_win.py [--config ../config/farm_config.yaml]
"""
import argparse
import json
import logging
import threading
import time

import serial
import socketio
import yaml

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("radio_bridge")

MAX_BYTES = 1900


def load_config(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


class RadioBridge:
    def __init__(self, cfg):
        self._cfg     = cfg
        self._ser     = None
        self._sio     = socketio.Client(reconnection=True, reconnection_delay=3)
        self._tx_lock = threading.Lock()
        self._running = False

        self._rx_count   = 0  # robot → lab PC
        self._tx_count   = 0  # lab PC → robot
        self._last_rx_msg = None
        self._last_tx_msg = None
        self._connected  = False
        self._stats_lock = threading.Lock()

    def _open_serial(self):
        r = self._cfg["radio"]
        self._ser = serial.Serial(
            port="COM8", baudrate=r["baud_rate"],
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
                with self._stats_lock:
                    self._rx_count += 1
                    self._last_rx_msg = msg
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
                with self._stats_lock:
                    self._tx_count += 1
                    self._last_tx_msg = payload
            except serial.SerialTimeoutException:
                log.error("Serial TX: Write timeout (CTS not asserted)")
            except Exception as e:
                log.error(f"Serial TX: {e}")

    def _stats_loop(self):
        prev_rx = 0
        prev_tx = 0
        while self._running:
            time.sleep(1)
            with self._stats_lock:
                rx_total = self._rx_count
                tx_total = self._tx_count
                last_rx  = self._last_rx_msg
                last_tx  = self._last_tx_msg
                connected = self._connected

            d_rx = rx_total - prev_rx
            d_tx = tx_total - prev_tx
            prev_rx = rx_total
            prev_tx = tx_total

            status = "CONNECTED" if connected else "DISCONNECTED"
            print(
                f"\n{'─'*55}\n"
                f"  Bridge Status : {status}\n"
                f"  Robot → Lab   : {d_rx:3d} msg/s  (total {rx_total})\n"
                f"  Lab → Robot   : {d_tx:3d} msg/s  (total {tx_total})\n"
                f"  Last RX       : {json.dumps(last_rx) if last_rx else '—'}\n"
                f"  Last TX       : {json.dumps(last_tx) if last_tx else '—'}\n"
                f"{'─'*55}",
                flush=True,
            )

    def _setup_sio(self):
        sio = self._sio

        @sio.on("connect", namespace="/bridge")
        def on_connect():
            with self._stats_lock:
                self._connected = True
            log.info("Connected to Lab PC")

        @sio.on("disconnect", namespace="/bridge")
        def on_disconnect():
            with self._stats_lock:
                self._connected = False
            log.warning("Disconnected from Lab PC")

        @sio.on("to_robot", namespace="/bridge")
        def on_command(data):
            self._send_to_robot(data)

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

        threading.Thread(target=self._stats_loop, daemon=True).start()

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
    parser.add_argument("--config", default="../config/farm_config.yaml")
    args = parser.parse_args()
    cfg = load_config(args.config)
    RadioBridge(cfg).run()


if __name__ == "__main__":
    main()