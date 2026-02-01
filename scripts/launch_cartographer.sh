#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

MODE="${1:-real}"

echo "[start_cartographer.sh] Mode: $MODE"

# Start cartographer in background
source "$WORKSPACE_DIR/cartographer_ws/install/setup.bash"
ros2 launch "$WORKSPACE_DIR/cartographer_ws/launch/cartographer.launch.py" mode:=$MODE &

# Wait for cartographer to initialize
sleep 5

# Start map saver with mode parameter
python3 "$WORKSPACE_DIR/cartographer_ws/scripts/save_map.py" --mode $MODE