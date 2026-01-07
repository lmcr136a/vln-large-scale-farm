#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

SESSION="navi_system"

tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION -n "livox_driver"

tmux send-keys -t $SESSION:0 "source $WORKSPACE_DIR/cartographer_ws/install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_launch.py" C-m

sleep 2

tmux new-window -t $SESSION:1 -n "cartographer"
tmux send-keys -t $SESSION:1 "source $WORKSPACE_DIR/cartographer_ws/install/setup.bash && ros2 launch $WORKSPACE_DIR/cartographer_ws/launch/livox_cartographer.launch.py" C-m

tmux new-window -t $SESSION:2 -n "janus"
tmux send-keys -t $SESSION:2 "/usr/local/bin/janus" C-m

tmux new-window -t $SESSION:3 -n "map_saver"
tmux send-keys -t $SESSION:3 "source $WORKSPACE_DIR/cartographer_ws/install/setup.bash && python3 $WORKSPACE_DIR/cartographer_ws/scripts/save_map.py" C-m

tmux new-window -t $SESSION:4 -n "control_server"
tmux send-keys -t $SESSION:4 "cd $WORKSPACE_DIR/tools_janus && python3 control_server.py" C-m

tmux attach-session -t $SESSION
