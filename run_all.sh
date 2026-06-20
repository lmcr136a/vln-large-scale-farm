#!/bin/bash

SCRIPT_DIR="launch_files"
WORKSPACE_DIR=$SCRIPT_DIR

MODE="${1:-real}"
SESSION="vln"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🚀 Starting Navigation System in $MODE mode"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

tmux kill-session -t $SESSION 2>/dev/null

# Window 0: Sensors (LiDAR | Xsens IMU | GPS) - 3 horizontal panes
tmux new-session -d -s $SESSION -n "Sensors"
tmux send-keys -t $SESSION:0 "bash $SCRIPT_DIR/launch_robosense.sh" C-m

tmux split-window -t $SESSION:0 -v
tmux send-keys -t $SESSION:0 "bash $SCRIPT_DIR/launch_xsens.sh $MODE" C-m

tmux split-window -t $SESSION:0 -v
tmux send-keys -t $SESSION:0 "python3 scripts/rtk_gps_node.py" C-m

tmux select-layout -t $SESSION:0 even-horizontal

# Window 1: SLAM
tmux new-window -t $SESSION:1 -n "Claude"
tmux send-keys -t $SESSION:1 "claude" C-m

# Window 2: Control Panel
tmux new-window -t $SESSION:2 -n "Main"
tmux send-keys -t $SESSION:2 "bash $SCRIPT_DIR/control_panel_jetson.sh" C-m

# Window 5: Obstacle Detection
tmux new-window -t $SESSION:3 -n "---"
tmux send-keys -t $SESSION:3 "echo ''" C-m

# Window 6: Status Info
tmux new-window -t "$SESSION:4" -n "---"
tmux send-keys -t "$SESSION:4" "clear; printf '%s\n' \
'' \
'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━' \
' Navigation System Started in $MODE mode' \
'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━' \
'' \
'Windows:' \
'  0: Sensors (LiDAR | Xsens IMU | GPS)' \
'  1: Claude' \
'  2: Main' \
'' \
'Control Panel:' \
'  http://localhost:8000/control.html' \
'' \
'' \
' PLEASE CHECK CONTROL CONFIG YAML' \
'' \
'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━' \
''" C-m

tmux attach-session -t "$SESSION"