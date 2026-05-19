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

echo "🚗 Setting up CAN interface..."

echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend >/dev/null || true

sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up

ip -details link show can0

echo "🦾 Launching Scout Mini base node..."
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/ros2_ws/scout_ws/install/setup.bash"

ros2 launch scout_base scout_mini_base.launch.py &
SCOUT_PID=$!

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