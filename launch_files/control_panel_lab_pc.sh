#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cleanup() {
    echo ""
    echo "🛑 Stopping remote server..."
    [ ! -z "$SERVER_PID" ] && kill $SERVER_PID 2>/dev/null
    sleep 1
    pkill -f "remote_server.py"
    echo "✅ Stopped"
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "🖥️  Starting Lab PC remote server..."
cd "$WORKSPACE_DIR"
python3 -u tools_control_panel/lab_pc/remote_server.py &
SERVER_PID=$!

sleep 1

TAILSCALE_IP=$(tailscale ip 2>/dev/null | head -1)
LOCAL_IP=$(hostname -I | awk '{print $1}')

echo ""
echo "✅ Remote server running"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
[ ! -z "$TAILSCALE_IP" ] && echo "  Tailscale : http://$TAILSCALE_IP:8000"
echo "  Local     : http://$LOCAL_IP:8000"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Press Ctrl+C to stop"
wait