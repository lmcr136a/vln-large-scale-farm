# tools_control_panel

Web-based control panel and autonomous driving system for SCI-Scout agricultural robot.

## Architecture

```
tools_control_panel/
├── autonomous/          # Path following logic
│   ├── autonomous_driving.py   # Generator-based closed-loop controller
│   └── autonomous_mode.py      # ROS2 node: pose tracking, drive loop, recording
├── config/
│   ├── farm_config.yaml        # Robot and system configuration
│   └── mission.json            # Saved path (start + waypoints + isLoop)
├── jetson/              # Runs on Jetson (robot onboard)
│   ├── jetson_main.py          # Entry point: wires all components
│   ├── commander.py            # Priority command handler (estop > manual > auto)
│   ├── internet_comm.py        # Socket.IO client to lab PC (Tailscale)
│   ├── radio_comm.py           # Serial radio link to local PC
│   ├── station_uploader.py     # SCP upload on autonomous run completion
│   ├── telemetry_node.py       # ROS2 node: battery, sensors, disk
│   └── auto_nav_logger.py      # Session log files in auto_nav_log/
├── lab_pc/              # Runs on lab PC
│   ├── remote_server.py        # Flask + Socket.IO relay server
│   └── server_to_panel.py      # Sysmon + saved-map push to browser
├── mapping/
│   └── save_map_glim.py        # Occupancy map builder from GLIM point clouds
├── web/                 # Browser panel
│   ├── control.html            # Main control panel
│   ├── control.js              # Map, robot display, commands
│   ├── path_plan.html          # Path planning page
│   ├── path_plan.js            # Click-to-set waypoints
│   └── styles.css              # Panel styles
└── README.md
```

## Data layout (outside tools_control_panel/)

```
vln-large-scale-farm/
├── tools_control_panel/
├── output_glim/         # map_latest.png, map_state.json
├── data/
│   ├── auto_nav/        # Camera + rosbag recordings per session
│   └── ...
└── auto_nav_log/        # yymmdd-HHMMSS.log per autonomous session
```

## Key flows

### Map pipeline
`save_map_glim.py` (Jetson) → writes `output_glim/map_latest.png` every 60 s (if map changed)  
`jetson_main.map_watch_loop` → sends PNG via internet when quality ≠ low  
`remote_server.inet_map_frame` → saves as `saved_map.png` + forwards to browser  

### Pose pipeline
`/glim_ros/localized_curr_pose` → `pose_corrector_cb` (applies LiDAR extrinsic) → `/corrected_pose`  
→ `autonomous_mode` (navigation) + `internet.send_pose` → lab PC → browser  

### Autonomous driving
1. Set path in `/path_plan.html` (click waypoints → Save Mission)
2. Click **Run Now** on control panel (or wait for schedule)
3. Jetson: `commander` → `AutonomousController._drive_loop`  
4. Camera + rosbag recording auto-starts → saved to `data/auto_nav/<timestamp>/`  
5. Log saved to `auto_nav_log/<timestamp>.log`

### Run / Resume
Autonomous driving is started manually from the control panel:
- **RUN** — drive the saved path from the first waypoint. Press again to stop
  (soft stop of path following).
- **Resume** — after stopping mid-drive, continue from the waypoint nearest the
  robot's current position (not the first one).

## Configuration

Key fields in `farm_config.yaml`:
- `lidar_extrinsics.rotation: [0, 0, 180]` — backward-facing LiDAR yaw offset
- `ros2.topics.raw_pose` / `pose` — raw GLIM → corrected pose topic
- `map.update_interval: 60` — map send interval (seconds)
- `autonomous.position_tolerance: 0.3` — waypoint arrival threshold (metres)

## Running

**Jetson:**
```bash
python3 tools_control_panel/jetson/jetson_main.py
```

**Lab PC:**
```bash
python3 tools_control_panel/lab_pc/remote_server.py
```

**Mapping (separate terminal on Jetson):**
```bash
python3 tools_control_panel/mapping/save_map_glim.py
```

**Path planning:** Open `http://<lab-pc-ip>:8000/path_plan.html`  
**Control panel:** Open `http://<lab-pc-ip>:8000`