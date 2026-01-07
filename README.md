# Agricultural Robot Navigation System

ROS2-based autonomous navigation system for agricultural robots using Livox LiDAR, Cartographer SLAM, and real-time video streaming via Janus WebRTC.

## System Overview

This system integrates:
- **Livox MID-360 LiDAR** for 3D mapping and localization
- **Cartographer SLAM** for real-time occupancy mapping
- **Janus Gateway** for low-latency WebRTC video streaming
- **Scout Robot Platform** for autonomous navigation

## Prerequisites

- Ubuntu 22.04 (Jetson or x86_64)
- ROS2 Humble
- Python 3.10+
- CUDA (for Jetson platform)

## Repository Structure
```
vln-large-scale-farm/
├── README.md                          # This file
├── dependencies/                      # Installation scripts
│   ├── install_janus.sh              # Janus WebRTC gateway setup
│   └── install_ros2_cartographer.sh  # ROS2 workspace setup
├── cartographer_ws/                   # ROS2 Cartographer workspace
│   ├── src/
│   │   └── livox_ros_driver2/        # Livox LiDAR driver
│   ├── launch/
│   │   └── livox_cartographer.launch.py
│   ├── config/
│   │   └── livox_3d.lua              # Cartographer configuration
│   ├── scripts/
│   │   └── save_map.py               # Map visualization & saving
│   └── output/                        # Generated maps (gitignored)
│       └── figures/
├── janus_streaming/                   # WebRTC streaming server
│   ├── control.html                   # Web control interface
│   ├── control_server.py              # Flask control server
│   └── autonomous_driving.py          # Navigation logic
├── scout_control/                     # Scout robot ROS2 control
│   ├── install_scout_ros2.sh
│   └── ros2_ws/
├── scripts/                           # System execution scripts
│   ├── start_livox_driver.sh
│   ├── start_cartographer.sh
│   ├── start_janus.sh
│   ├── start_map_saver.sh
│   ├── start_control_server.sh
│   └── start_all.sh                   # Launch all services via tmux
└── 3_zed-data-tools/                  # ZED camera utilities
```

## Installation

### 1. Clone Repository
```bash
cd ~
git clone <repository-url> vln-large-scale-farm
cd vln-large-scale-farm
```

### 2. Install Janus Gateway
```bash
cd dependencies
chmod +x install_janus.sh
./install_janus.sh
```

This script will:
- Install all system dependencies
- Build libnice and Janus from source
- Configure streaming plugin for OBSBot camera
- Install Python dependencies (Flask, Flask-SocketIO)

### 3. Build ROS2 Cartographer Workspace
```bash
cd dependencies
chmod +x install_ros2_cartographer.sh
./install_ros2_cartographer.sh
```

This script will:
- Install ROS2 Humble dependencies
- Build Livox SDK2 and ROS2 driver
- Build Cartographer ROS2 packages
- Install Python dependencies (numpy, Pillow, PyYAML, tf2-ros)

### 4. Build Scout Control (Optional)
```bash
cd scout_control
chmod +x install_scout_ros2.sh
./install_scout_ros2.sh
```

## Usage

### Quick Start (All Services)

Launch all services in a single tmux session:
```bash
cd ~/vln-large-scale-farm
./scripts/start_all.sh
```

**Tmux Controls:**
- `Ctrl+b` then `0-4`: Switch between windows
  - Window 0: Livox Driver
  - Window 1: Cartographer
  - Window 2: Janus
  - Window 3: Map Saver
  - Window 4: Control Server
- `Ctrl+b` then `d`: Detach from session (keeps running in background)
- `tmux attach -t navi_system`: Re-attach to session
- `Ctrl+b` then `&`: Kill current window

### Manual Launch (Individual Services)

#### Terminal 1: Livox Driver
```bash
./scripts/start_livox_driver.sh
```

#### Terminal 2: Cartographer SLAM
```bash
./scripts/start_cartographer.sh
```

#### Terminal 3: Janus WebRTC Server
```bash
./scripts/start_janus.sh
```

#### Terminal 4: Map Saver
```bash
./scripts/start_map_saver.sh
```

#### Terminal 5: Control Server (Optional)
```bash
./scripts/start_control_server.sh
```

### Accessing the System

1. **WebRTC Video Stream**: Open browser to `http://<robot-ip>:8088/control.html`
2. **Control Interface**: Flask server runs on `http://<robot-ip>:5000`
3. **Generated Maps**: Check `cartographer_ws/output/map_latest.png`

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

### Janus Streaming Settings

Edit `/opt/janus/etc/janus/janus.plugin.streaming.jcfg`:
```
obsbot: {
  type = "rtp"
  id = 1
  videoport = 8004
  videopt = 100
  videocodec = "h264"
}
```

## Troubleshooting

### LiDAR Not Connecting
```bash
# Check network interface
ifconfig

# Test LiDAR connectivity
ping 192.168.1.1XX

# Verify ROS2 topics
ros2 topic list | grep livox
```

### Cartographer Not Building Map
```bash
# Check transform tree
ros2 run tf2_tools view_frames

# Monitor point cloud data
ros2 topic echo /livox/lidar --no-arr
```

### Janus Streaming Issues
```bash
# Check Janus logs
journalctl -u janus -f

# Verify streaming config
cat /opt/janus/etc/janus/janus.plugin.streaming.jcfg
```

## Output Files

- `cartographer_ws/output/map_latest.png`: Latest occupancy grid visualization
- `cartographer_ws/output/map_latest.yaml`: Map metadata (resolution, origin, robot pose)
- `cartographer_ws/output/figures/building_N.png`: Map snapshots over time

## Development

### Adding New Launch Files

Place custom launch files in `cartographer_ws/launch/` and update scripts accordingly.

### Modifying Map Saver

Edit `cartographer_ws/scripts/save_map.py` to customize visualization (colors, trajectory length, save interval).

### Extending Control Server

Edit `janus_streaming/control_server.py` and `autonomous_driving.py` for custom navigation behaviors.

## System Requirements

- **RAM**: 8GB minimum (16GB recommended)
- **Storage**: 20GB free space
- **Network**: Gigabit Ethernet for LiDAR connection
- **GPU**: CUDA-capable GPU for Jetson platform

## Known Issues

- LiDAR coordinate frame may need calibration for accurate pose estimation
- High-speed motion can cause SLAM drift - tune Cartographer parameters
- WebRTC streaming requires low-latency network (< 50ms RTT recommended)

## License

[Add your license here]

## Contributors

Evelyn Nahyeon - UCLA MAE Department

## References

- [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [Cartographer ROS2](https://github.com/ros2/cartographer_ros)
- [Janus Gateway](https://github.com/meetecho/janus-gateway)
