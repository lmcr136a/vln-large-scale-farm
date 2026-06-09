#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
GLIM_WS="$WORKSPACE_DIR/ros2_ws/glim_ws"
PATCHWORK_CONFIG="$GLIM_WS/src/patchwork-plusplus_ws/ros/config/actual.yaml"

cleanup() {
    kill "$FILTER_PC_PID" 2>/dev/null
    wait "$FILTER_PC_PID" 2>/dev/null
}
trap cleanup EXIT

python3 "$WORKSPACE_DIR/scripts/vertical_structure_filter.py" &
FILTER_PC_PID=$!

source /opt/ros/humble/setup.bash
source "$GLIM_WS/install/setup.bash"

if [ ! -f "$PATCHWORK_CONFIG" ]; then
  echo "Patchwork++ config not found: $PATCHWORK_CONFIG"
  exit 1
fi

if ! ros2 pkg prefix patchworkpp >/dev/null 2>&1; then
  echo "ROS package patchworkpp not found."
  echo "Build it first:"
  echo "  cd $GLIM_WS"
  echo "  colcon build --packages-select patchworkpp"
  echo "  source install/setup.bash"
  exit 1
fi

echo "Patchwork++ config: $PATCHWORK_CONFIG"

ros2 launch patchworkpp patchworkpp.launch.py \
  config_file:="$PATCHWORK_CONFIG" \
  visualize:=false