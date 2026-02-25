#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

MODE="${1:-real}"

echo "SLAM Mode: $MODE"
source "$WORKSPACE_DIR/livox_driver2_ws/install/setup.bash"


# source "$WORKSPACE_DIR/fast_lio_ws/install/setup.bash"
# ros2 launch fast_lio mapping.launch.py config_path:=/home/nahyeon/box/vln-large-scale-farm/fast_lio_ws/src/FAST_LIO/config config_file:=1scout_livox.yaml

source "$WORKSPACE_DIR/glim_ws/install/setup.bash"

# Start glim_converters in background
ros2 run glim_converters glim_converters &
CONVERTER_PID=$!
sleep 1
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/home/nahyeon/box/vln-large-scale-farm/glim_ws/config

trap "kill $CONVERTER_PID 2>/dev/null" EXIT