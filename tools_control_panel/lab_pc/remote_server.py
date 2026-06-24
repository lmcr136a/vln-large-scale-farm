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
import math
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

logging.getLogger("werkzeug").setLevel(logging.ERROR)

CFG_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../config/farm_config.yaml"))


class RemoteServer:
    def __init__(self, config_path=CFG_PATH):
        self.cfg = load_config(config_path)
        self._config_path = os.path.expanduser(config_path)
        cfg_dir = os.path.dirname(self._config_path)
        self._paths_file    = os.path.join(cfg_dir, self.cfg["paths"].get("paths_file",   "paths.json"))
        self._mission_file  = os.path.join(cfg_dir, self.cfg["paths"].get("mission_file", "mission.json"))
        self._path_nodes       = self._load_paths()
        self._path_mode        = False
        self._bridge_connected  = False
        self._inet_connected    = False
        self._inet_sid          = None
        self._latest_landmarks: dict = {"landmarks": []}
        # On a fresh server start the on-disk saved_map.png is from a PREVIOUS
        # session, so don't push it to clients — wait for this session's first
        # live map_frame. Avoids the old SLAM map flashing before the real one.
        self._have_fresh_map    = False

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
            r = send_file(os.path.join(web_dir, "control.js"), mimetype="application/javascript")
            r.headers["Cache-Control"] = "no-store"
            return r

        @self.app.route("/styles.css")
        def serve_css():
            r = send_file(os.path.join(web_dir, "styles.css"), mimetype="text/css")
            r.headers["Cache-Control"] = "no-store"
            return r

        @self.app.route("/path_plan.html")
        def serve_path_plan():
            return send_file(os.path.join(web_dir, "path_plan.html"))

        @self.app.route("/path_plan.js")
        def serve_path_plan_js():
            r = send_file(os.path.join(web_dir, "path_plan.js"), mimetype="application/javascript")
            r.headers["Cache-Control"] = "no-store"
            return r


        @self.app.route("/mission", methods=["GET", "POST"])
        def handle_mission():
            from flask import jsonify, request as req
            if req.method == "GET":
                try:
                    with open(self._mission_file) as f:
                        return jsonify(json.load(f))
                except FileNotFoundError:
                    return jsonify({"start": None, "waypoints": []})
                except Exception as e:
                    log.error(f"Mission read error: {e}")
                    return jsonify({"start": None, "waypoints": []})
            data      = req.get_json(force=True)
            start     = data.get("start")
            waypoints = data.get("waypoints", [])
            is_loop   = data.get("isLoop", False)
            try:
                with open(self._mission_file, "w") as f:
                    json.dump({"start": start, "waypoints": waypoints, "isLoop": is_loop}, f, indent=2)
                log.info(f"Mission saved: {1 + len(waypoints)} pts, loop={is_loop}")
            except Exception as e:
                log.error(f"Mission save error: {e}")
                return jsonify({"ok": False})
            return jsonify({"ok": True})

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

        @self.app.route("/landmarks", methods=["GET"])
        def handle_landmarks():
            from flask import jsonify
            return jsonify(self._latest_landmarks)

    # ── Panel events (browser ↔ server) ──────────────────────────────────────

    def _register_panel_events(self):
        sio = self.sio

        @sio.on("connect")
        def on_connect():
            log.info(f"Panel connected: {request.sid}")
            if self._path_nodes:
                sio.emit("path_loaded", {"waypoints": self._path_nodes}, to=request.sid)
            self._send_map_to_client(request.sid)
            if self._latest_landmarks.get("landmarks"):
                sio.emit("landmarks_updated", self._latest_landmarks, to=request.sid)

        @sio.on("disconnect")
        def on_disconnect():
            if request.sid == self._inet_sid:
                # Jetson internet connection dropped — clear so _to_robot falls back to radio
                self._inet_connected = False
                self._inet_sid       = None
                log.warning("Jetson internet disconnected")
            else:
                log.info(f"Panel disconnected: {request.sid}")

        @sio.on("heartbeat")
        def on_heartbeat():
            pass

        @sio.on("estop")
        def on_estop():
            self._to_robot({"cmd": "estop"})

        @sio.on("command")
        def on_command(data):
            # Generic command passthrough from browser → Jetson.
            # Handles: restart_window, and any future commands.
            if not isinstance(data, dict):
                return
            log.info(f"[cmd] {data.get('cmd')} from panel")
            self._to_robot(data)

        @sio.on("keydown")
        def on_keydown(key: str):
            try:
                VEL = load_config(self._config_path).get("autonomous", self.cfg["autonomous"])
            except Exception:
                VEL = self.cfg["autonomous"]
            lin = VEL.get("linear_speed",  0.3)
            ang = VEL.get("angular_speed", 0.3)
            vx, vz = 0.0, 0.0
            if key == "ArrowUp":      vx =  lin
            elif key == "ArrowDown":  vx = -lin
            elif key == "ArrowLeft":  vz =  ang
            elif key == "ArrowRight": vz = -ang
            if vx or vz:
                log.info(f"[cmd] velocity vx={vx} vz={vz}  bridge={self._bridge_connected}  inet={self._inet_connected}")
                self._to_robot({"cmd": "velocity", "vx": vx, "vz": vz})

        @sio.on("keyup")
        def on_keyup(_key):
            self._to_robot({"cmd": "velocity", "vx": 0.0, "vz": 0.0})

        @sio.on("start_autonomous")
        def on_start(data):
            waypoints = self._resolve_waypoints(data)
            if not waypoints:
                log.warning("start_autonomous: no waypoints available, ignoring")
                return
            # resume=False → drive the whole path from the first waypoint.
            self._to_robot({"cmd": "continue", "waypoints": waypoints, "resume": False})

        @sio.on("resume_autonomous")
        def on_resume(data):
            waypoints = self._resolve_waypoints(data)
            if not waypoints:
                log.warning("resume_autonomous: no waypoints available, ignoring")
                return
            # resume=True → continue from start_index if given (1-based in UI,
            # already 0-based here), else the waypoint nearest the robot.
            msg = {"cmd": "continue", "waypoints": waypoints, "resume": True}
            if isinstance(data, dict) and data.get("start_index") is not None:
                msg["start_index"] = int(data["start_index"])
            self._to_robot(msg)

        @sio.on("stop_autonomous")
        def on_stop():
            self._to_robot({"cmd": "stop_auto"})

        @sio.on("start_recording")
        def on_start_recording(dirname):
            if not dirname:
                return
            self._to_robot({"cmd": "start_recording", "dirname": str(dirname)})
            log.info(f"start_recording → Jetson: {dirname}")

        @sio.on("stop_recording")
        def on_stop_recording():
            self._to_robot({"cmd": "stop_recording"})
            log.info("stop_recording → Jetson")

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
            sio.emit("path_saved", {"count": len(self._path_nodes)})

        @sio.on("set_safety_enabled")
        def on_set_safety_enabled(data):
            enabled = bool(data.get("enabled", True)) if isinstance(data, dict) else bool(data)
            self._to_robot({"cmd": "set_safety_enabled", "enabled": enabled})
            log.info(f"set_safety_enabled → Jetson: {enabled}")

        @sio.on("flip_heading")
        def on_flip_heading():
            self._to_robot({"cmd": "flip_heading"})
            log.info("flip_heading → Jetson")

        @sio.on("set_quality")
        def on_set_quality(data):
            level = data.get("level", "low")
            self._to_robot_internet({"type": "quality", "level": level})

    # ── Bridge events (Local PC radio relay → Lab PC) ─────────────────────────

    def _register_bridge_events(self):
        sio = self.sio

        @sio.on("connect", namespace="/bridge")
        def bridge_connect():
            self._bridge_connected = True
            log.info(f"Radio bridge connected: {request.sid}")

        @sio.on("disconnect", namespace="/bridge")
        def bridge_disconnect():
            self._bridge_connected = False
            log.warning("Radio bridge disconnected")

        @sio.on("from_robot", namespace="/bridge")
        def from_robot(msg):
            mtype = msg.get("type")
            if mtype == "telemetry":
                self._forward_telemetry(msg["data"])
            elif mtype == "pose":
                # Real-time pose over radio (used when internet is down).
                self.sio.emit("robot_pose", self._correct_yaw({
                    "x":   msg.get("x"),
                    "y":   msg.get("y"),
                    "yaw": msg.get("yaw"),
                }), namespace="/")
            elif mtype == "radio_frame":
                sio.emit("radio_frame", {
                    "camera": msg.get("camera"),
                    "data":   msg.get("data"),
                }, namespace="/")
            elif mtype == "event":
                event = msg.get("event")
                if event == "robot_pose":
                    return
                sio.emit(event, msg.get("data", {}), namespace="/")

    # ── Internet events (Jetson direct) ───────────────────────────────────────

    def _register_internet_events(self):
        sio = self.sio

        @sio.on("jetson_hello")
        def inet_hello(data=None):
            self._inet_connected = True
            self._inet_sid       = request.sid
            log.info(f"Jetson internet connected: {request.sid}")



        @sio.on("ping_rtt")
        def inet_ping(data):
            sio.emit("pong_rtt", {}, to=request.sid)

        @sio.on("telemetry")
        def inet_telemetry(data):
            self._forward_telemetry(data)

        @sio.on("robot_event")
        def inet_event(data):
            event = data.get("event")
            if event == "robot_pose":
                return
            payload = data.get("data", {})
            if event == "landmarks_updated":
                self._latest_landmarks = payload
            sio.emit(event, payload, namespace="/")

        @sio.on("rgb_frame")
        def inet_rgb(data):
            cam = data.get("camera", "front")
            payload = {"data": data["data"]}
            if "t" in data:                 # frame sim-time → panel RGB↔GPS matching
                payload["t"] = data["t"]
            sio.emit(f"{cam}_frame", payload, namespace="/")

        @sio.on("pose")
        def inet_pose(data):
            sio.emit("robot_pose", self._correct_yaw(data), namespace="/")

        @sio.on("pointcloud")
        def inet_pointcloud(data):
            sio.emit("pointcloud_update", data, namespace="/")

        @sio.on("map_frame")
        def inet_map_frame(data):
            import base64
            meta = data.get("meta", {})
            try:
                png_bytes = base64.b64decode(data.get("data", ""))
                save_dir  = self._monitor._saved_map_dir
                with open(os.path.join(save_dir, "saved_map.png"), "wb") as f:
                    f.write(png_bytes)
                with open(os.path.join(save_dir, "saved_map_state.json"), "w") as f:
                    json.dump(meta, f)
            except Exception as e:
                log.error(f"map_frame save: {e}")
            self._have_fresh_map = True
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

        @sio.on("to_robot")
        def inet_to_robot(data):
            sio.emit("command", data)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _correct_yaw(self, data: dict) -> dict:
        ext = self.cfg.get("lidar_extrinsics", {})
        yaw_offset = math.radians(ext.get("rotation", [0, 0, 0])[2])
        if yaw_offset == 0.0:
            return data
        corrected = data["yaw"] + yaw_offset
        corrected = (corrected + math.pi) % (2 * math.pi) - math.pi
        return {**data, "yaw": corrected}

    def _send_map_to_client(self, sid: str):
        import base64
        # Skip until this session has produced a real map — the file on disk may
        # be a stale map from a previous run (would flash before the live one).
        if not self._have_fresh_map:
            return
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

    def _resolve_waypoints(self, data: dict) -> list:
        """Waypoints from the panel payload, or fall back to the saved mission
        file (start + waypoints, plus the loop-closing return to start)."""
        waypoints = (data or {}).get("waypoints", [])
        if waypoints:
            return waypoints
        try:
            with open(self._mission_file) as f:
                mission = json.load(f)
            start   = mission.get("start")
            wps     = mission.get("waypoints", [])
            is_loop = mission.get("isLoop", False)
            waypoints = ([start] if start else []) + wps
            if is_loop and start:
                waypoints.append(start)
            log.info(f"loaded {len(waypoints)} waypoints from mission file")
        except Exception as e:
            log.warning(f"mission load failed: {e}")
        return waypoints

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
        """Forward all telemetry fields to panel clients."""
        # process_status is the current field; tmux_status kept for legacy Jetson builds.
        proc = data.get("process_status") or data.get("tmux_status")
        self.sio.emit("robot_telemetry", {
            "battery":               data.get("batt", -1),
            "sensors":               data.get("sensors", {}),
            "storage_pct":           data.get("storage_pct", -1),
            "mode":                  data.get("mode", "idle"),
            "estop":                 data.get("estop", False),
            "wifi":                  data.get("wifi", "—"),
            "internet":              data.get("internet", False),
            "radio_quality":         data.get("radio_quality"),
            "radio_rtt_ms":          data.get("radio_rtt_ms"),
            "internet_quality":      data.get("internet_quality"),
            "internet_rtt_ms":       data.get("internet_rtt_ms"),
            "safety_status":         data.get("safety_status"),
            "gps_status":            data.get("gps_status"),
            "process_status":        proc,
            "process_status_detail": data.get("process_status_detail"),
            "net":                   data.get("net"),
            "base_status":           data.get("base_status"),
            "system_ready":          data.get("system_ready"),
        }, namespace="/")

    def _to_robot(self, cmd: dict):
        if self._bridge_connected:
            self.sio.emit("to_robot", cmd, namespace="/bridge")
        if self._inet_connected and self._inet_sid:
            self.sio.emit("command", cmd, to=self._inet_sid)
        if not self._bridge_connected and not self._inet_connected:
            log.warning(f"No robot connection — command dropped: {cmd.get('cmd')}")

    def _to_robot_internet(self, msg: dict):
        mtype = msg.get("type")
        if mtype == "quality":
            self.sio.emit("quality", msg)
        else:
            self.sio.emit("command", msg)

    def _push_config_update(self, updates: dict):
        try:
            with open(self._config_path) as f:
                cfg = yaml.safe_load(f)
            self._deep_update(cfg, updates)
            with open(self._config_path, "w") as f:
                yaml.dump(cfg, f)
            self._deep_update(self.cfg, updates)  # keep in-memory cfg in sync
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