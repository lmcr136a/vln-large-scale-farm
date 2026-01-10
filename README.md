# Agricultural Robot Navigation System

ROS2-based autonomous navigation system for Scout robots.

## System Overview

This system integrates:
- **Livox MID-360 LiDAR** for 3D mapping and localization
- **Cartographer SLAM** for real-time occupancy mapping
- **ROS2 Image Topics** for live camera streaming
- **Scout Robot Platform** for autonomous navigation

## Prerequisites

- Ubuntu 22.04 (Jetson or x86_64)
- ROS2 Humble
- Python 3.10+
- NumPy < 2.0 (for cv_bridge compatibility)

## Repository Structure
```
vln-large-scale-farm/
├── README.md                          # This file
├── install.sh                         # Main installation script
├── cartographer_ws/                   # ROS2 Cartographer workspace
│   ├── src/
│   │   └── livox_ros_driver2/        # Livox LiDAR driver
│   ├── launch/
│   │   └── livox_cartographer.launch.py
│   ├── config/
│   │   └── livox_3d.lua              # Cartographer configuration
│   ├── scripts/
│   │   └── save_map.py               # Map visualization & saving
│   ├── install_ros2_cartographer.sh  # Cartographer setup
│   └── output/                        # Generated maps (gitignored)
│       └── figures/
├── tools_control_panel/               # Web-based control interface
│   ├── control.html                   # Web control UI
│   ├── control_server.py              # Flask server with ROS2 integration
│   ├── autonomous_driving.py          # Navigation logic
│   └── run_control_panel.sh           # Launch script
├── tools_scout_control/               # Scout robot ROS2 control
│   ├── install_scout_ros2.sh         # Scout driver installation
│   ├── jetson-gs_usb-kernel-builder.sh
│   └── ros2_ws/                       # Scout ROS2 workspace
├── tools_zed/                         # ZED camera utilities
│   └── data_svo_sync.py              # Data recording tools
├── scripts/                           # System execution scripts
│   ├── start_livox_driver.sh
│   ├── start_cartographer.sh
│   ├── start_map_saver.sh
│   └── start_all.sh                   # Launch all services via tmux
└── data/                              # Recorded data (gitignored)
```

## Installation

### 1. Clone Repository
```bash
git clone https://github.com/lmcr136a/vln-large-scale-farm.git
cd vln-large-scale-farm
```

### 2. Install System Dependencies
```bash
# Install NumPy 1.x (required for cv_bridge)
pip3 install "numpy<2"

# Install other Python dependencies
pip3 install flask flask_socketio opencv-python psutil pyyaml
```

### 3. Build ROS2 Cartographer Workspace
```bash
cd cartographer_ws
chmod +x install_ros2_cartographer.sh
./install_ros2_cartographer.sh
```

This script will:
- Install ROS2 Humble dependencies
- Build Livox SDK2 and ROS2 driver
- Build Cartographer ROS2 packages
- Install Python dependencies (numpy<2, Pillow, PyYAML, tf2-ros)

### 4. Build Scout Control
```bash
cd tools_scout_control
chmod +x install_scout_ros2.sh
./install_scout_ros2.sh
```

**Note:** If you use Anaconda/Miniconda, deactivate conda before building.

## Usage

### Quick Start - Control Panel

Launch the web-based control interface:
```bash
cd ~/vln-large-scale-farm/tools_control_panel
./run_control_panel.sh
```

This will start:
- Scout robot control node
- Flask control server with ROS2 integration
- HTTP server for web interface

Access the control panel at: `http://<robot-ip>:8000/control.html`

### Manual Launch (Individual Services)

#### Terminal 1: Livox Driver
```bash
./scripts/start_livox_driver.sh
```

#### Terminal 2: Cartographer SLAM
```bash
./scripts/start_cartographer.sh
```

#### Terminal 3: Map Saver
```bash
./scripts/start_map_saver.sh
```

### Accessing the System

1. **Web Control Interface**: `http://<robot-ip>:8000/control.html`
   - Live camera stream from ROS2 `/rgb` topic
   - Real-time map visualization
   - Manual robot control (arrow keys)
   - Autonomous navigation with waypoint planning

2. **Control Server API**: `http://<robot-ip>:5000`
   - WebSocket for real-time communication
   - Map updates and system monitoring

3. **Generated Maps**: `cartographer_ws/output/map_latest.png`

## Web Control Interface Features

- **Live Video Stream**: Real-time RGB camera feed from ROS2 topics
- **Interactive Map**: Click to set waypoints for autonomous navigation
- **Manual Control**:
  - Arrow keys: Move robot
  - W/A/S/D: PTZ camera control
  - Z/X: Zoom control
  - </> and []: Speed adjustments
- **Autonomous Mode**: Create navigation paths and execute autonomous driving
- **System Monitoring**: CPU, memory, disk usage, WiFi status

## Configuration

### Livox LiDAR Network Settings

Edit `cartographer_ws/src/livox_ros_driver2/config/MID360_config.json`:
```json
{
  "lidar_configs": [
    {
      "ip": "192.168.1.1XX",
      "pcl_data_type": 1,
      "pattern_mode": 0
    }
  ]
}
```

### Cartographer Parameters

Edit `cartographer_ws/config/livox_3d.lua` to tune SLAM performance:
- `max_range`: Maximum LiDAR range
- `num_accumulated_range_data`: Number of scans to accumulate
- `voxel_filter_size`: Point cloud downsampling resolution

### Camera Topic Configuration

Edit `tools_control_panel/control_server.py` to change the RGB topic:
```python
rgb_sub = node.create_subscription(Image, '/rgb', rgb_callback, 10)
```

## Output Files

- `cartographer_ws/output/map_latest.png`: Latest occupancy grid visualization
- `cartographer_ws/output/map_latest.yaml`: Map metadata (resolution, origin, robot pose)
- `cartographer_ws/output/figures/building_N.png`: Map snapshots over time
- `data/`: Recorded sensor data (ZED camera, LiDAR)

## Troubleshooting

### NumPy Version Error
If you see `AttributeError: _ARRAY_API not found`, downgrade NumPy:
```bash
pip3 install "numpy<2"
```

### Camera Stream Not Showing
Check if the `/rgb` topic is publishing:
```bash
ros2 topic list | grep -i rgb
ros2 topic echo /rgb --once
```

### Control Panel Not Starting
Verify all dependencies are installed:
```bash
pip3 list | grep -E "flask|opencv|psutil"
```

## References

- [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [Cartographer ROS2](https://github.com/ros2/cartographer_ros)
- [Scout Mobile Robot](https://github.com/agilexrobotics/scout_ros2)
- [Flask-SocketIO](https://flask-socketio.readthedocs.io/)