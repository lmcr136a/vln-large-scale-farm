#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

source "$WORKSPACE_DIR/cartographer_ws/install/setup.bash"
python3 "$WORKSPACE_DIR/cartographer_ws/scripts/save_map.py"
