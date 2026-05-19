#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
TOOLS_DIR="$WORKSPACE_DIR/tools_control_panel"

cleanup() {
    echo ""
    echo "🛑 Stopping all services..."
    [ ! -z "$SCOUT_PID" ] && kill $SCOUT_PID 2>/dev/null
    [ ! -z "$JETSON_PID" ] && kill $JETSON_PID 2>/dev/null
    sleep 1
    pkill -f "scout_mini_base"
    pkill -f "jetson_main.py"
    echo "✅ Stopped"
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "🚗 Setting up CAN interface..."
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 up type can bitrate 500000

echo "🦾 Launching Scout Mini base node..."
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/ros2_ws/scout_ws/install/setup.bash"
echo "$WORKSPACE_DIR/ros2_ws/scout_ws/install/setup.bash"
ros2 launch scout_base scout_mini_base.launch.py &
SCOUT_PID=$!
sleep 2

# Clear the window state file so restart logic starts fresh from window 2
rm -f /tmp/jetson_main_window

echo ""
echo "🤖 Starting Jetson agent..."
cd "$WORKSPACE_DIR"
python3 -u tools_control_panel/jetson/jetson_main.py &
JETSON_PID=$!

echo ""
echo "✅ Jetson agent running"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Config : $TOOLS_DIR/config/farm_config.yaml"
echo "  Radio  : /dev/ttyTHS1"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Press Ctrl+C to stop"
wait