# tools_control_panel

Web-based control panel and GPS-guided autonomous driving system for the
SCI-Scout agricultural robot (large-scale farm row navigation).

The robot runs on a Jetson; a lab PC runs a relay server; an operator drives and
monitors everything from a browser panel. The link is Tailscale (internet) with a
serial radio fallback, so telemetry and the safety banners keep working even when
the internet is down.

## Architecture

```
tools_control_panel/
├── autonomous/          # Path-following logic
│   ├── autonomous_driving.py   # Generator-based closed-loop cross-track controller
│   └── autonomous_mode.py      # ROS2 node: pose tracking, drive loop, recording,
│                               #   RTK gating, heading establishment + stuck recovery
├── config/
│   ├── farm_config.yaml        # Robot and system configuration
│   ├── mission.json            # Saved path (start + waypoints + isLoop)
│   └── crop_field.json         # Crop-field rectangle (ref_x, y_south, y_north, rotation_deg)
├── gps/
│   └── gps_localizer.py        # GPS→ENU pose + heading (establish / maintain / reset)
├── jetson/              # Runs on the Jetson (robot onboard)
│   ├── jetson_main.py          # Entry point: wires every component together
│   ├── commander.py            # Priority command handler (safety > estop > manual > auto)
│   ├── internet_comm.py        # Socket.IO client to the lab PC (Tailscale)
│   ├── radio_comm.py           # Serial radio link (low-bandwidth fallback)
│   ├── station_uploader.py     # SCP upload triggered at a docking station on run completion
│   ├── telemetry_node.py       # ROS2 node: battery, per-sensor/-camera liveness, disk, RTK
│   └── auto_nav_logger.py      # Per-session log files in auto_nav_log/
├── lab_pc/              # Runs on the lab PC
│   ├── remote_server.py        # Flask + Socket.IO relay (browser ⇄ Jetson)
│   └── server_to_panel.py      # Sysmon + saved-map push to the browser
├── mapping/
│   └── save_map_glim.py        # Occupancy map builder from GLIM point clouds
├── perception/          # Landmark detection / mapping (YOLO + storage)
├── sensor/
│   ├── all_zeds.py             # ZED camera publishers
│   ├── safety_guard.py         # Obstacle override (pause auto + back away)
│   ├── safety_checker.py       # Per-zone safety state for the panel
│   └── recorder.py             # Camera + rosbag recording
├── web/                 # Browser panel
│   ├── control.html / control.js   # Map, robot display, alerts, commands
│   ├── path_plan.html / path_plan.js  # Click-to-set waypoints
│   └── styles.css
└── README.md
```

## Data layout (repository root)

```
vln-large-scale-farm/
├── run_all.sh              # Launches the whole system in a tmux session ("vln")
├── deploy/                 # Boot autostart (systemd service + install script)
├── scripts/uploader.py     # Box upload daemon (runs in tmux window 3)
├── tools_control_panel/
├── output_glim/            # map_latest.png, map_state.json
├── data/
│   ├── auto_nav/           # Camera + rosbag recordings, one folder per session
│   └── <site>/<session>/   # Datasets the uploader pushes to Box
└── auto_nav_log/           # yymmdd-HHMMSS.log per autonomous session
```

## Running

### Boot autostart (recommended)

Install once (needs sudo for the systemd unit + a passwordless rule for the CAN bus):

```bash
sudo bash deploy/install_autostart.sh
```

After this the system starts on every boot. The `vln.service` unit runs
`run_all.sh`, which creates a detached tmux session (no terminal needed). Useful
commands:

```bash
tmux attach -t vln          # view the running windows (detach with Ctrl-b then d)
systemctl status vln        # service status
journalctl -u vln -b        # boot logs
sudo systemctl start vln    # start now
sudo systemctl stop vln     # stop (kills the tmux session)
```

### Manual launch

```bash
./run_all.sh                # creates the tmux session and attaches
```

tmux windows created by `run_all.sh`:

| Window | Name     | Contents |
|--------|----------|----------|
| 0      | Sensors  | LiDAR \| Xsens IMU \| RTK GPS (3 panes) |
| 1      | Claude   | `claude -c` |
| 2      | Main     | `control_panel_jetson.sh` (Scout base, safety checker, `jetson_main.py`) |
| 3      | Uploader | `scripts/uploader.py` (Box upload + verified delete) |
| 4      | Status   | Static info screen |

**Lab PC** (relay server):

```bash
python3 tools_control_panel/lab_pc/remote_server.py
```

**Path planning:** open `http://<lab-pc-ip>:8000/path_plan.html`
**Control panel:** open `http://<lab-pc-ip>:8000`

## Key flows

### Pose / heading pipeline

`/gps/fix` + IMU → `gps_localizer` → ENU pose with a confirmed heading →
`/corrected_pose` → consumed by `autonomous_mode` and forwarded to the browser.

Heading handling in `gps_localizer.py`:
- **Establish** — the absolute heading is unknown at start; the first ~1 m of
  forward travel confirms it from the GPS course.
- **Maintain** — straight driving snaps the heading to the GPS course (clearing
  gyro drift); in-place rotation falls back to gyro integration.
- **Reset** — `reset_heading()` re-confirms from fresh travel; used by the
  autonomous stuck-recovery (below).

### Autonomous driving

1. Set a path in `path_plan.html` (click waypoints → Save Mission).
2. Press **RUN** on the control panel.
3. Jetson: `commander` → `AutonomousController._drive_loop` →
   `autonomous_driving.run` (cross-track line follower).
4. Camera + rosbag recording auto-starts → `data/auto_nav/<timestamp>/`.
5. A session log is written to `auto_nav_log/<timestamp>.log`.

The controller **holds in place rather than aborting** when conditions are not
safe to drive, and never silently ends the run:
- **RTK gating** — with `require_rtk_fixed`, it waits/holds until the GPS
  solution is accurate (RTK Fixed, or accurate Float), then continues.
- **Safety override** — `safety_guard` can pause path following and back away
  from an obstacle, then resume from the same point.
- **Stuck recovery** — if the robot makes no real progress for ~20 s while
  actively driving (the GPS N/S heading was lost or flipped and the follower
  deadlocked), it nudges ~0.5 m, biased toward the N/S row direction, to
  re-acquire the heading and carries on — instead of sitting until someone
  presses Resume.

### RUN / Resume

- **RUN** — drive the saved path from the first waypoint. Press again to stop
  (soft stop of path following).
- **Resume** — after a stop, continue from the waypoint nearest the robot's
  current position (never one already passed). An optional "from pt" index
  overrides this.
- The run state survives a **browser refresh**: the server replays it on
  reconnect, so the button stays in sync and a refresh never stops the robot.

### Map pipeline

`save_map_glim.py` writes `output_glim/map_latest.png` → `jetson_main` sends it
over the internet → `remote_server` saves `saved_map.png` and forwards it to the
browser. The crop field (`crop_field.json`) renders as a fixed parallelogram with
green/white row stripes generated west of `ref_x`.

### Data upload

`scripts/uploader.py` (tmux window 3) is a daemon that waits for WiFi, uploads
each `data/<site>/<session>/` folder to Box with `rclone copy`, then verifies the
upload with `rclone check` (per-file hashes) and **deletes the local folder only
after that check passes**. `station_uploader.py` is a separate, optional SCP
upload triggered when the robot reaches a docking station at run completion.

## Panel alerts

Hard-to-miss red, pulsing banners at the top of the control panel:
- **BASE NOT RESPONDING / BASE FAULT** — Scout CAN link down or a base error
  code (from telemetry, so it shows over radio too).
- **CAMERA DOWN** — any expected camera (front / right / back / left) stopped
  streaming, including Argus open failures. Driven by per-camera liveness in
  telemetry (`sensors.zed_<name>`).

## Configuration

Key fields in `config/farm_config.yaml`:
- `lidar_extrinsics.rotation` — LiDAR mounting yaw offset.
- `autonomous.position_tolerance` — waypoint arrival threshold (m).
- `autonomous.require_rtk_fixed` — hold for accurate RTK before/while driving.
- `autonomous.stop_rotate_threshold` / cross-track gains — line-follower tuning.
- `recording.zed_cameras` — the expected cameras (names drive the CAMERA DOWN alert).
- `map.update_interval` — map send interval (s).
- `upload.*` — `station_uploader.py` destination (host, ssh_user, ssh_key,
  remote_path, station_pose, station_radius). The Box daemon `scripts/uploader.py`
  is configured by constants at the top of that file.
