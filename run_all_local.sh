#!/bin/bash

SESSION="vln"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🚀 Starting Navigation System"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION -n "Web"
tmux send-keys -t $SESSION:0 "python3 tools_control_panel/lab_pc/remote_server.py" C-m

tmux new-window -t $SESSION:1 -n "Radio"
tmux send-keys -t $SESSION:1 "python3 tools_control_panel/local_pc/radio_bridge_linux.py" C-m

tmux attach-session -t "$SESSION"