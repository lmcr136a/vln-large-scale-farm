#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Mode selection: real or sim
MODE="${1:-real}"

SESSION="vln"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🚀 Starting Navigation System in $MODE mode"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Kill existing session
tmux kill-session -t $SESSION 2>/dev/null

# Create new session
tmux new-session -d -s $SESSION -n "lidar"

# Window 0: Sensor (Livox or Isaac Sim)
if [ "$MODE" = "sim" ]; then
    tmux send-keys -t $SESSION:0 "echo '⚠️  Start Isaac Sim manually, then run:'" C-m
    tmux send-keys -t $SESSION:0 "echo 'ros2 launch isaac_cartographer_launch isaac_cartographer.launch.py use_sim_time:=true'" C-m
else
    tmux send-keys -t $SESSION:0 "source $WORKSPACE_DIR/cartographer_ws/install/setup.bash" C-m
    tmux send-keys -t $SESSION:0 "ros2 launch livox_ros_driver2 msg_MID360_launch.py" C-m
fi

sleep 3

# Window 1: Cartographer + Map Saver 
tmux new-window -t $SESSION:1 -n "carto"
tmux send-keys -t $SESSION:1 "bash $SCRIPT_DIR/launch_cartographer.sh $MODE" C-m

sleep 3

# Window 2: Control Panel
tmux new-window -t $SESSION:2 -n "control"
tmux send-keys -t $SESSION:2 "bash $SCRIPT_DIR/launch_control_panel.sh" C-m

# Attach to session
tmux new-window -t "$SESSION:3" -n "-"
tmux send-keys -t "$SESSION:3" "clear; printf '%s\n' \
'' \
'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━' \
' Navigation System Started in $MODE mode' \
'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━' \
'' \
'Windows:' \
'  0: Lidar Sensor ($MODE)' \
'  1: Cartographer + Map Saver' \
'  2: Control Panel' \
'' \
'Control Panel:' \
'  http://100.78.219.75:8000/control.html' \
'' \
'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━' \
''" C-m

tmux attach-session -t "$SESSION"