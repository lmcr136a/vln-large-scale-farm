#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$WORKSPACE_DIR/ros2_ws/glim_ws/install/setup.bash"
export LD_LIBRARY_PATH=$WORKSPACE_DIR/ros2_ws/glim_ws/install/glim_localizer/lib/glim_localizer:$LD_LIBRARY_PATH
echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"

MODE="${1:-real}"
echo "SLAM Mode: $MODE Work Space Dir: $WORKSPACE_DIR"

ros2 run glim_ros glim_rosnode \
  --ros-args \
  -p config_path:=$WORKSPACE_DIR/ros2_ws/glim_ws/config_outdoor

trap "kill $CONVERTER_PID 2>/dev/null" EXIT