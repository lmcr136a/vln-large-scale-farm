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
tmux new-session -d -s $SESSION -n "lidar"

# Window 0: Sensor (Livox or Isaac Sim)
if [ "$MODE" = "sim" ]; then
    tmux send-keys -t $SESSION:0 "echo '⚠️  Start Isaac Sim manually, then run:'" C-m
    tmux send-keys -t $SESSION:0 "echo 'ros2 launch isaac_cartographer_launch isaac_cartographer.launch.py use_sim_time:=true'" C-m
else
    tmux send-keys -t $SESSION:0 "source $WORKSPACE_DIR/livox_driver2_ws/install/setup.bash" C-m
    tmux send-keys -t $SESSION:0 "ros2 launch livox_ros_driver2 msg_MID360_launch.py" C-m
fi

sleep 1

# Window 1: Cartographer + Map Saver 
tmux new-window -t $SESSION:1 -n "slam"
tmux send-keys -t $SESSION:1 "bash $SCRIPT_DIR/launch_slam.sh $MODE" C-m

sleep 1

# Window 2: Control Panel
tmux new-window -t $SESSION:2 -n "control"
tmux send-keys -t $SESSION:2 "bash $SCRIPT_DIR/launch_control_panel.sh" C-m

# Window 3: 2D Map Saver
tmux new-window -t $SESSION:3 -n "2dmap"
tmux send-keys -t $SESSION:3 "python3 fast_lio_ws/save_map.py" C-m

# Window 4: Rosbag
tmux new-window -t $SESSION:4 -n "rosbag"

# Window 5: Status Info
tmux new-window -t "$SESSION:5" -n "---"
tmux send-keys -t "$SESSION:5" "clear; printf '%s\n' \
'' \
'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━' \
' Navigation System Started in $MODE mode' \
'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━' \
'' \
'Windows:' \
'  0: Lidar Sensor ($MODE)' \
'  1: Cartographer + Map Saver' \
'  2: Control Panel' \
'  3: 2D Map Saver' \
'  4: Rosbag' \
'' \
'Control Panel:' \
'  http://100.78.219.75:8000/control.html' \
'' \
'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━' \
''" C-m

tmux attach-session -t "$SESSION"