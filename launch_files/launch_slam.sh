#!/bin/bash
# sudo pkill -9 -f cartographer

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

MODE="${1:-real}"

echo "[start_cartographer.sh] Mode: $MODE"

# Start cartographer in background
source "$WORKSPACE_DIR/livox_driver2_ws/install/setup.bash"
source "$WORKSPACE_DIR/fast_lio_ws/install/setup.bash"
ros2 launch fast_lio mapping.launch.py config_path:=/home/nahyeon/box/vln-large-scale-farm/fast_lio_ws/src/FAST_LIO/config config_file:=1scout_livox.yaml

sleep 1

# Start map saver with mode parameter
python3 "$WORKSPACE_DIR/fast_lio_ws/save_map.py"