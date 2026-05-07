#!/bin/bash

SCRIPT_DIR="launch_files"
WORKSPACE_DIR=$SCRIPT_DIR

# Mode selection: real or sim
MODE="${1:-real}"

SESSION="vln"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🚀 Starting Navigation System in $MODE mode"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Kill existing session
tmux kill-session -t $SESSION 2>/dev/null

# Create new session
tmux new-session -d -s $SESSION -n "LiDAR"
tmux send-keys -t $SESSION:0 "bash $SCRIPT_DIR/launch_robosense.sh" C-m

# Window 1: lidar driver
tmux new-window -t $SESSION:1 -n "SLAM"
tmux send-keys -t $SESSION:1 "bash $SCRIPT_DIR/launch_slam.sh $MODE" C-m

# Window 2: Control Panel
tmux new-window -t $SESSION:2 -n "Web"
tmux send-keys -t $SESSION:2 "bash $SCRIPT_DIR/control_panel_jetson.sh" C-m

# Window 3: 2D Map Saver
tmux new-window -t $SESSION:3 -n "2Dmap"
tmux send-keys -t $SESSION:3 "python3 tools_control_panel/save_map_glim.py" C-m

# Window 4: Obstacle Detection
tmux new-window -t $SESSION:4 -n "O.D."
tmux send-keys -t $SESSION:4 "python3 tools_scout_control/safety_checker.py" C-m

# Window 5: Xsens RTK & IMU
tmux new-window -t $SESSION:5 -n "xsens"
tmux send-keys -t $SESSION:5 "bash $SCRIPT_DIR/launch_xsens.sh $MODE" C-m


# Window 6: Status Info
tmux new-window -t "$SESSION:6" -n "---"
tmux send-keys -t "$SESSION:6" "clear; printf '%s\n' \
'' \
'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━' \
' Navigation System Started in $MODE mode' \
'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━' \
'' \
'Windows:' \
'  0: LiDAR' \
'  1: SLAM' \
'  2: WEB Control Panel' \
'  3: 2D Map Saver' \
'  4: Obstacle Detection' \
'  5: Xsens' \
'  6: Here' \
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