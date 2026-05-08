"""
Lab PC remote server.
Replaces control_server.py — no ROS2 dependency.
Data sources:
  - /bridge namespace: telemetry relayed from Local PC radio bridge (always-on)
  - /internet namespace: direct from Jetson when internet available (RGB, pointcloud)
  - /  namespace: browser web panel clients
"""
import json
import logging
import os
import sys
import threading
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config_loader import load_config

from flask import Flask, request, send_file
from flask_socketio import SocketIO, emit

from server_to_panel import ServerToPanel

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("remote_server")

CFG_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../config/farm_config.yaml"))


class RemoteServer:
    def __init__(self, config_path=CFG_PATH):
        self.cfg = load_config(config_path)
        self._config_path = os.path.expanduser(config_path)
        cfg_dir = os.path.dirname(self._config_path)
        self._paths_file = os.path.join(cfg_dir, self.cfg["paths"].get("paths_file", "paths.json"))
        self._path_nodes = self._load_paths()
        self._path_mode = False

        self.app = Flask(__name__, template_folder="../lab_pc")
        self.sio = SocketIO(
            self.app,
            cors_allowed_origins="*",
            async_mode="threading",
            ping_interval=self.cfg["server"]["ping_interval"],
            ping_timeout=self.cfg["server"]["ping_timeout"],
            max_http_buffer_size=self.cfg["server"].get("max_http_buffer_size", 10_000_000),
            allow_upgrades=False,
        )

        self._register_routes()
        self._register_panel_events()
        self._register_bridge_events()
        self._register_internet_events()

        self._monitor = ServerToPanel(self.sio, self.cfg)

    # ── HTTP ──────────────────────────────────────────────────────────────────

    def _register_routes(self):
        web_dir = os.path.join(os.path.dirname(__file__), "../web")

        @self.app.route("/")
        @self.app.route("/control.html")
        def serve_panel():
            return send_file(os.path.join(web_dir, "control.html"))

        @self.app.route("/control.js")
        def serve_js():
            return send_file(os.path.join(web_dir, "control.js"), mimetype="application/javascript")

        @self.app.route("/styles.css")
        def serve_css():
            return send_file(os.path.join(web_dir, "styles.css"), mimetype="text/css")

        @self.app.route("/saved_map.png")
        def serve_saved_map():
            path = os.path.join(self._monitor._saved_map_dir, "saved_map.png")
            if not os.path.exists(path):
                return ("No saved map yet", 404)
            return send_file(path, mimetype="image/png", max_age=0, conditional=False)

        @self.app.route("/config", methods=["GET", "POST"])
        def handle_config():
            from flask import jsonify, request as req
            if req.method == "GET":
                return jsonify(load_config(self._config_path))
            updates = req.get_json(force=True)
            self._push_config_update(updates)
            return jsonify({"ok": True})

    # ── Panel events (browser ↔ server) ──────────────────────────────────────

    def _register_panel_events(self):
        sio = self.sio

        @sio.on("connect")
        def on_connect():
            log.info(f"Panel connected: {request.sid}")
            if self._path_nodes:
                sio.emit("path_loaded", {"waypoints": self._path_nodes}, to=request.sid)
            self._send_map_to_client(request.sid)

        @sio.on("disconnect")
        def on_disconnect():
            log.info(f"Panel disconnected: {request.sid}")

        @sio.on("heartbeat")
        def on_heartbeat():
            pass

        @sio.on("estop")
        def on_estop():
            self._to_robot({"cmd": "estop"})

        # control.js emits socket.emit('keydown', key) — key is a raw string
        @sio.on("keydown")
        def on_keydown(key: str):
            VEL = self.cfg["autonomous"]
            vx, vz = 0.0, 0.0
            if key == "ArrowUp":      vx =  VEL["linear_speed"]
            elif key == "ArrowDown":  vx = -VEL["linear_speed"]
            elif key == "ArrowLeft":  vz =  VEL["angular_speed"]
            elif key == "ArrowRight": vz = -VEL["angular_speed"]
            if vx or vz:
                self._to_robot({"cmd": "velocity", "vx": vx, "vz": vz})

        @sio.on("keyup")
        def on_keyup(_key):
            self._to_robot({"cmd": "velocity", "vx": 0.0, "vz": 0.0})

        @sio.on("start_autonomous")
        def on_start(data):
            waypoints = data.get("waypoints", [])
            self._to_robot({"cmd": "continue", "waypoints": waypoints})

        @sio.on("stop_autonomous")
        def on_stop():
            self._to_robot({"cmd": "stop_auto"})

        @sio.on("map_clicked")
        def on_map_clicked(data):
            if self._path_mode:
                self._path_nodes.append({"x": data["world_x"], "y": data["world_y"]})
                sio.emit("path_updated", {"waypoints": self._path_nodes})

        @sio.on("toggle_path_mode")
        def on_toggle_path_mode():
            self._path_mode = not self._path_mode
            if self._path_mode:
                self._path_nodes = []
            sio.emit("path_mode_state", {"active": self._path_mode})

        @sio.on("save_path")
        def on_save_path():
            self._save_paths(self._path_nodes)
            self._push_config_update({"autonomous": {"waypoints": self._path_nodes}})
            sio.emit("path_saved", {"count": len(self._path_nodes)})

        @sio.on("set_quality")
        def on_set_quality(data):
            level = data.get("level", "low")
            # Tell Jetson to adjust streaming quality
            self._to_robot_internet({"type": "quality", "level": level})

    # ── Bridge events (Local PC radio relay → Lab PC) ─────────────────────────

    def _register_bridge_events(self):
        sio = self.sio

        @sio.on("connect", namespace="/bridge")
        def bridge_connect():
            log.info(f"Radio bridge connected: {request.sid}")

        @sio.on("disconnect", namespace="/bridge")
        def bridge_disconnect():
            log.warning("Radio bridge disconnected")

        @sio.on("from_robot", namespace="/bridge")
        def from_robot(msg):
            mtype = msg.get("type")
            if mtype == "telemetry":
                self._forward_telemetry(msg["data"])
            elif mtype == "event":
                sio.emit(msg["event"], msg.get("data", {}), namespace="/")

    # ── Internet events (Jetson direct) ───────────────────────────────────────

    def _register_internet_events(self):
        sio = self.sio

        @sio.on("connect", namespace="/internet")
        def inet_connect():
            log.info(f"Jetson internet connected: {request.sid}")

        @sio.on("disconnect", namespace="/internet")
        def inet_disconnect():
            log.warning("Jetson internet disconnected")

        @sio.on("ping_rtt", namespace="/internet")
        def inet_ping(data):
            sio.emit("pong_rtt", {}, to=request.sid, namespace="/internet")

        # Events emitted by internet_comm.py
        @sio.on("telemetry", namespace="/internet")
        def inet_telemetry(data):
            self._forward_telemetry(data)

        @sio.on("robot_event", namespace="/internet")
        def inet_event(data):
            sio.emit(data["event"], data.get("data", {}), namespace="/")

        @sio.on("rgb_frame", namespace="/internet")
        def inet_rgb(data):
            cam = data.get("camera", "front")
            event = "front_frame" if cam == "front" else "back_frame"
            sio.emit(event, {"data": data["data"]}, namespace="/")

        @sio.on("pose", namespace="/internet")
        def inet_pose(data):
            sio.emit("robot_pose", data, namespace="/")

        @sio.on("pointcloud", namespace="/internet")
        def inet_pointcloud(data):
            sio.emit("pointcloud_update", data, namespace="/")

        @sio.on("map_frame", namespace="/internet")
        def inet_map_frame(data):
            import base64
            meta = data.get("meta", {})
            # Persist to disk so the fallback path has an up-to-date saved map
            try:
                png_bytes = base64.b64decode(data.get("data", ""))
                save_dir  = self._monitor._saved_map_dir
                with open(os.path.join(save_dir, "saved_map.png"), "wb") as f:
                    f.write(png_bytes)
                with open(os.path.join(save_dir, "saved_map_state.json"), "w") as f:
                    json.dump(meta, f)
            except Exception as e:
                log.error(f"map_frame save: {e}")
            self._monitor.notify_jetson_map()
            self._monitor._map_version += 1
            sio.emit("map_updated", {
                "version":    self._monitor._map_version,
                "resolution": float(meta.get("resolution", 0.1)),
                "origin_x":   float(meta.get("origin_x",  0.0)),
                "origin_y":   float(meta.get("origin_y",  0.0)),
                "width":      int(meta.get("img_width",   0)),
                "height":     int(meta.get("img_height",  0)),
                "rot_angle":  float(meta.get("rot_angle", 0.0)),
                "image_data": data.get("data", ""),
            }, namespace="/")

        # Commands from panel → Jetson via internet
        @sio.on("to_robot", namespace="/internet")
        def inet_to_robot(data):
            sio.emit("command", data, namespace="/internet")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _send_map_to_client(self, sid: str):
        """Send the latest saved map directly to a single client (e.g. on reconnect)."""
        import base64
        save_dir   = self._monitor._saved_map_dir
        png_path   = os.path.join(save_dir, "saved_map.png")
        state_path = os.path.join(save_dir, "saved_map_state.json")
        if not os.path.exists(png_path) or not os.path.exists(state_path):
            return
        try:
            with open(png_path, "rb") as f:
                b64 = base64.b64encode(f.read()).decode()
            with open(state_path) as f:
                meta = json.load(f)
            self._monitor._map_version += 1
            self.sio.emit("map_updated", {
                "version":    self._monitor._map_version,
                "resolution": float(meta.get("resolution", 0.1)),
                "origin_x":   float(meta.get("origin_x",  0.0)),
                "origin_y":   float(meta.get("origin_y",  0.0)),
                "width":      int(meta.get("img_width",   0)),
                "height":     int(meta.get("img_height",  0)),
                "rot_angle":  float(meta.get("rot_angle", 0.0)),
                "image_data": b64,
            }, to=sid, namespace="/")
        except Exception as e:
            log.error(f"send map on connect: {e}")

    def _load_paths(self) -> list:
        try:
            with open(self._paths_file) as f:
                return json.load(f)
        except FileNotFoundError:
            return []
        except Exception as e:
            log.error(f"paths.json read error: {e}")
            return []

    def _save_paths(self, waypoints: list):
        try:
            with open(self._paths_file, "w") as f:
                json.dump(waypoints, f, indent=2)
        except Exception as e:
            log.error(f"paths.json write error: {e}")

    def _forward_telemetry(self, data: dict):
        """Unpack telemetry dict and emit status fields to panel clients."""
        self.sio.emit("robot_telemetry", {
            "battery":     data.get("batt", -1),
            "sensors":     data.get("sensors", {}),
            "storage_pct": data.get("storage_pct", -1),
            "mode":        data.get("mode", "idle"),
            "estop":       data.get("estop", False),
        }, namespace="/")

    def _to_robot(self, cmd: dict):
        """Send command via radio bridge AND internet (whichever is connected)."""
        self.sio.emit("to_robot", cmd, namespace="/bridge")
        self.sio.emit("command", cmd, namespace="/internet")

    def _to_robot_internet(self, msg: dict):
        """Send command to Jetson via direct internet connection."""
        mtype = msg.get("type")
        if mtype == "quality":
            self.sio.emit("quality", msg, namespace="/internet")
        else:
            self.sio.emit("command", msg, namespace="/internet")

    def _push_config_update(self, updates: dict):
        try:
            with open(self._config_path) as f:
                cfg = yaml.safe_load(f)    # raw read — preserve relative paths on disk
            self._deep_update(cfg, updates)
            with open(self._config_path, "w") as f:
                yaml.dump(cfg, f)
            log.info(f"Config saved: {list(updates.keys())}")
        except Exception as e:
            log.error(f"Config save error: {e}")
        self._to_robot({"cmd": "config_update", "config": updates})

    @staticmethod
    def _deep_update(base: dict, updates: dict):
        for k, v in updates.items():
            if isinstance(v, dict) and isinstance(base.get(k), dict):
                RemoteServer._deep_update(base[k], v)
            else:
                base[k] = v

    def run(self):
        import threading
        threading.Thread(target=self._monitor.monitor_loop, daemon=True).start()
        log.info(f"Remote server on :{self.cfg['server']['port']}")
        self.sio.run(
            self.app,
            host=self.cfg["server"]["host"],
            port=self.cfg["server"]["port"],
            allow_unsafe_werkzeug=True,
            use_reloader=False,
            log_output=False,
        )


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--config", default=CFG_PATH)
    args = p.parse_args()
    RemoteServer(args.config).run()