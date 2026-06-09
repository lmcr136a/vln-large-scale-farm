#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$WORKSPACE_DIR/ros2_ws/glim_ws/install/setup.bash"
export LD_LIBRARY_PATH=$WORKSPACE_DIR/ros2_ws/glim_ws/install/glim_localizer/lib/glim_localizer:$LD_LIBRARY_PATH
echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"

MODE="${1:-fargo}"
CONFIG_DIR="$WORKSPACE_DIR/ros2_ws/glim_ws/config_$MODE"

if [ ! -d "$CONFIG_DIR" ]; then
  echo "Error: config directory not found: $CONFIG_DIR"
  exit 1
fi

echo "SLAM Mode: $MODE  Config: $CONFIG_DIR  Work Space Dir: $WORKSPACE_DIR"

ros2 run glim_ros glim_rosnode \
  --ros-args \
  -p config_path:=$CONFIG_DIR

trap "kill $CONVERTER_PID 2>/dev/null" EXIT