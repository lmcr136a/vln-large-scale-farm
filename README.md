# Agricultural Robot Navigation System

ROS2-based autonomous navigation system with RGBD recording for Scout robots.

## System Overview

This system integrates:
- **Livox MID-360 LiDAR** for 3D mapping and localization
- **Cartographer 2D SLAM** for real-time occupancy mapping  
- **ZED X Stereo Cameras** (2x) for RGBD recording
- **Scout Robot Platform** for autonomous navigation
- **Web Control Interface** with live video streaming and RGBD control

## Prerequisites

- Ubuntu 22.04 (Jetson AGX Orin or x86_64)
- ROS2 Humble
- Python 3.10+
- NumPy < 2.0 (for cv_bridge compatibility)
- ZED SDK 5.1.2+ (for Jetson JP 6.1)
- Livox Driver

## Repository Structure

```
vln-large-scale-farm/
├── README.md                          # This file
├── cartographer_ws/                   # ROS2 Cartographer workspace
│   ├── src/
│   │   └── livox_ros_driver2/        # Livox LiDAR driver
│   ├── launch/
│   │   └── cartographer.launch.py    # 🆕 Unified launch (real/sim)
│   ├── config/
│   │   ├── livox_3d.lua              # Real robot config
│   │   └── isaacsim.lua              # Simulation config
│   ├── scripts/
│   │   └── save_map.py               # 🆕 Unified map saver
│   └── output/                        # Generated maps
│       ├── map_latest.png
│       ├── map_latest.yaml
│       └── figures/
│           └── YYYYMMDD_HHMMSS/      # 🆕 Timestamped maps
├── tools_control_panel/               # Web-based control interface
│   ├── control.html                   # 🆕 Web UI with RGBD controls
│   ├── control_server.py              # 🆕 Flask + RGBD API
│   └── autonomous_driving.py          # Navigation logic
├── tools_rgbd_cameras/                # 🆕 ZED X camera recording
│   └── zed_dual_recorder.py          # Dual camera recorder
├── tools_scout_control/               # Scout robot control
│   ├── install_scout_ros2.sh
│   └── ros2_ws/
├── scripts/                           # 🆕 Unified launch scripts
│   ├── launch_all.sh                  # Main launcher (real/sim)
│   ├── launch_livox_driver.sh         # Sensor driver
│   ├── launch_cartographer.sh         # SLAM + map saver
│   └── launch_control_panel.sh        # Web interface
└── output_dir/                        # Recorded data
    ├── front/
    │   ├── rgb/
    │   └── depth/
    └── back/
```

## Installation

### 1. Clone Repository
```bash
git clone https://github.com/lmcr136a/vln-large-scale-farm.git
cd vln-large-scale-farm
```

### 2. Build ROS2 Cartographer Workspace
```bash
cd cartographer_ws
chmod +x install_ros2_cartographer.sh
./install_ros2_cartographer.sh
```

### 3. Build Scout Control
```bash
cd tools_scout_control
chmod +x install_scout_ros2.sh
./install_scout_ros2.sh
```

## Quick Start

### Launch Full System

**Real Robot:**
```bash
cd ~/vln-large-scale-farm/scripts
./launch_all.sh real
```

**Simulation:**
```bash
cd ~/vln-large-scale-farm/scripts
./launch_all.sh sim
```

This launches:
- **Window 0**: LiDAR Driver (Livox MID-360 or Isaac Sim)
- **Window 1**: Cartographer SLAM + Map Saver
- **Window 2**: Web Control Panel

### Access Control Panel

Open browser: `http://<robot-ip>:8000/control.html`

Features:
- 📹 **RGBD Recording Control** - Start/Stop dual camera recording
- 🗺️ **Live Map Visualization** - Real-time SLAM mapping
- 🎮 **Robot Control** - Manual driving and autonomous navigation
- 📊 **System Monitoring** - CPU, memory, camera status


## Individual Component Launch

### Lidar Sensor Driver
```bash
# Real robot
./scripts/launch_livox_driver.sh real

# Simulation (manual Isaac Sim startup required)
Manually start Isaac-sim
```

### Cartographer + Map Saver
```bash
# Real robot
./scripts/launch_cartographer.sh real

# Simulation
./scripts/launch_cartographer.sh sim
```

### Control Panel
```bash
./scripts/launch_control_panel.sh
```

## Output Files

### Maps
- `cartographer_ws/output/map_latest.png` - Latest occupancy grid
- `cartographer_ws/output/map_latest.yaml` - Map metadata
- `cartographer_ws/output/figures/YYYYMMDD_HHMMSS/` - Timestamped map sequence

### RGBD Data
```
output_dir/
├── front/
│   ├── rgb/
│   │   ├── 1738446000000.png
│   │   └── ...
│   └── depth/
│       ├── 1738446000000.npy
│       └── ...
└── back/
    ├── rgb/ & depth/ (same structure)
```
- `output_dir/front/rgb/*.png` - Front camera RGB images
- `output_dir/front/depth/*.npy` - Front camera depth maps (float32)
- `output_dir/back/` - Back camera data (same structure)


---

**System Version**: v2.0  
**Last Updated**: February 2026  
**Tested on**: Jetson AGX Orin (JetPack 6.1, L4T 36.4)