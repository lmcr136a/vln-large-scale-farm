# Robot Control Server

Clean modular robot control system with integrated ZED cameras.

## Files

```
control_server.py       # Main server
control_config.yaml     # Configuration
server_to_panel.py      # System monitoring
map_creation.py         # Manual control
get_path.py             # Path planning
autonomous_mode.py      # Autonomous driving
zed_dual_camera.py      # ZED camera control
control.html            # Web UI
control.js              # Web logic
styles.css              # Web styles
```

## Usage

```bash
# Terminal 1: Control server (includes ZED cameras)
python3 -u control_server.py

# Terminal 2: Web server
python3 -m http.server 8000

# Browser
http://localhost:8000/control.html
```

## ZED Cameras

**Front camera:**
- Always streams to web panel (large view, left side)
- Saves to disk only when recording

**Back camera:**
- Always streams to web panel (small view, top-left corner)
- Saves to disk only when recording

## Recording

1. Enter directory name in panel
2. Click "Start" button
3. Both cameras save to disk and publish to ROS2: `~/box/vln-large-scale-farm/data/{dirname}/{timestamp}/`
4. Click "Stop" to stop recording (cameras continue streaming to web)

**Output:**
```
data/
└── {dirname}/
    └── {YYYYMMDD_HHMM}/
        ├── front/
        │   ├── rgb/*.png
        │   └── depth/*.npy
        └── back/
            ├── rgb/*.png
            └── depth/*.npy
```

## Controls

**Map creation:**
- W/A/S/D: PTZ pan/tilt
- Z/X: Zoom
- ↑↓←→: Robot movement
- < >: Linear speed
- [ ]: Angular speed

**Path mode:**
- Click map to add waypoints

**Autonomous:**
- Start/Stop buttons

## Dependencies

```bash
pip install flask flask-socketio psutil pyyaml opencv-python numpy
```

ROS2: rclpy, geometry_msgs, sensor_msgs, cv_bridge
ZED: pyzed