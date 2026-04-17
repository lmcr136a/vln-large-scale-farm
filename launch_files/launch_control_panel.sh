#!/bin/bash
# sudo pkill -9 -f control_server

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
TOOLS_DIR="$WORKSPACE_DIR/tools_control_panel"

cleanup() {
    echo ""
    echo "🛑 Stopping all services..."
    
    [ ! -z "$HTTP_PID" ] && kill $HTTP_PID 2>/dev/null
    [ ! -z "$CTRL_PID" ] && kill $CTRL_PID 2>/dev/null
    [ ! -z "$ROS_PID" ] && kill $ROS_PID 2>/dev/null
    
    sleep 1
    
    pkill -f "control_server.py"
    pkill -f "http.server 8000"
    pkill -f "scout_base"
    
    echo "✅ All services stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "🚗 Setting up CAN interface..."
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 up type can bitrate 500000

echo "🦾 Launching Scout Base Node..."
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/tools_scout_control/ros2_ws/install/setup.bash"
ros2 launch scout_base scout_base.launch.py &
ROS_PID=$!

sleep 1

echo "🕹️  Starting control server (with RGBD support)..."
cd "$TOOLS_DIR"
python3 -u control_server.py &
CTRL_PID=$!

# sleep 2

# echo "🌐 Starting HTTP server..."
# python3 -m http.server 8000 &
# HTTP_PID=$!

sleep 1

echo ""
echo "✅ All services started!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📱 Control Panel: http://100.78.219.75:8000/control.html"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Press Ctrl+C to stop all services"

wait
