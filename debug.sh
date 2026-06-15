#!/bin/bash

SCRIPT_DIR="launch_files"
WORKSPACE_DIR=$SCRIPT_DIR

MODE="${1:-real}"
SESSION="vln"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🚀 Starting Navigation System in $MODE mode"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION -n "Data"
tmux send-keys -t $SESSION:0 "cd data/fargo" C-m

# Window 1: SLAM
tmux new-window -t $SESSION:1 -n "SLAM"
tmux send-keys -t $SESSION:1 "bash $SCRIPT_DIR/launch_slam.sh fargo" C-m

# Window 2: Control Panel
tmux new-window -t $SESSION:2 -n "Main"
tmux send-keys -t $SESSION:2 "echo bash $SCRIPT_DIR/control_panel_jetson.sh" C-m

# Window 3: Ground segmentation
tmux new-window -t $SESSION:3 -n "Ground"
tmux send-keys -t $SESSION:3 "bash $SCRIPT_DIR/launch_ground_seg.sh" C-m

# Window 4: 2D Map Saver
tmux new-window -t $SESSION:4 -n "2Dmap"
tmux send-keys -t $SESSION:4 "python3 tools_control_panel/mapping/save_map_glim.py" C-m

# Window 5: Obstacle Detection
tmux new-window -t $SESSION:5 -n "---"
tmux send-keys -t $SESSION:5 "echo ''" C-m

# Window 6: Status Info
tmux new-window -t "$SESSION:6" -n "---"
tmux send-keys -t "$SESSION:6" "clear; printf '%s\n' \
'' \
'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━' \
' Navigation System Started in $MODE mode' \
'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━' \
'' \
'Windows:' \
'  0: Sensors (LiDAR | Xsens IMU | GPS)' \
'  1: SLAM' \
'  2: Main' \
'  3: 2D Map Saver' \
'  4: Obstacle Detection' \
'  5: Here' \
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