#!/bin/bash
TOOLS_DIR="$HOME/box/vln-large-scale-farm/tools_janus"
cp "$TOOLS_DIR"/*.{html,py} "$SCRIPT_DIR"/ 2>/dev/null || true # copy to script dir

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cleanup() {
    echo ""
    echo "🛑 Stopping all services..."
    
    # 각 프로세스를 부드럽게 종료 시도
    [ ! -z "$HTTP_PID" ] && kill $HTTP_PID 2>/dev/null
    [ ! -z "$CTRL_PID" ] && kill $CTRL_PID 2>/dev/null
    [ ! -z "$ROS_PID" ] && kill $ROS_PID 2>/dev/null
    
    sleep 2
    
    # 강제 종료
    [ ! -z "$HTTP_PID" ] && kill -9 $HTTP_PID 2>/dev/null
    [ ! -z "$CTRL_PID" ] && kill -9 $CTRL_PID 2>/dev/null
    [ ! -z "$ROS_PID" ] && kill -9 $ROS_PID 2>/dev/null
    
    # Python 프로세스 정리
    pkill -f "control_server.py"
    pkill -f "http.server 8000"
    
    echo "✅ All services stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "🚗 Setting up CAN interface..."
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000

echo "🦾 Launching Scout Base Node..."
ros2 launch scout_base scout_base.launch.py &
ROS_PID=$!

sleep 3

# echo "🛰️  Starting Janus..."
# sudo /opt/janus/bin/janus --configs-folder=/opt/janus/etc/janus &
# JANUS_PID=$!

# sleep 2

# echo "🎥 Starting GStreamer..."
# gst-launch-1.0 v4l2src device=/dev/video0 ! \
#         video/x-raw,width=640,height=360,framerate=15/1 ! \
#         nvvidconv ! \
#         nvv4l2h264enc insert-sps-pps=1 bitrate=300000 ! \
#         rtph264pay config-interval=1 pt=100 ! \
#         udpsink host=127.0.0.1 port=8004 sync=false async=false &
# GST_PID=$!

echo "🕹️  Starting control server..."
python3 -u "$SCRIPT_DIR/control_server.py" &
CTRL_PID=$!

sleep 2

echo "🌐 Starting HTTP server..."
cd "$SCRIPT_DIR"
python3 -m http.server 8000 &
HTTP_PID=$!

echo "✅ All services started!"
echo "📱 Open browser: http://$(hostname -I | awk '{print $1}'):8000/control.html"

wait