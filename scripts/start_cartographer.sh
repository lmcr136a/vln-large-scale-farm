#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

source "$WORKSPACE_DIR/cartographer_ws/install/setup.bash"
ros2 launch "$WORKSPACE_DIR/cartographer_ws/launch/livox_cartographer.launch.py"
