#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

MODE="${1:-real}"

echo "[launch_robosense.sh] Mode: $MODE"

if [ "$MODE" = "sim" ]; then
    echo "Isaac Sim mode: Livox driver is not used."
    echo "Please start Isaac Sim manually and run the simulation"
    exit 0
else
    echo "Starting Real Robot mode - Livox LiDAR..."
    source "$WORKSPACE_DIR/ros2_ws/robosense_ws/install/setup.bash"
    ros2 launch rslidar_sdk start.py
fi
