#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cleanup() {
    echo ""
    echo "🛑 Stopping radio bridge..."
    [ ! -z "$BRIDGE_PID" ] && kill $BRIDGE_PID 2>/dev/null
    sleep 1
    pkill -f "radio_bridge.py"
    echo "✅ Stopped"
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "📡 Starting radio bridge (Local PC → Lab PC)..."
cd "$WORKSPACE_DIR"
python3 -u tools_control_panel/local_pc/radio_bridge.py &
BRIDGE_PID=$!

sleep 1

echo ""
echo "✅ Radio bridge running"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Serial → Jetson radio"
echo "  WebSocket → Lab PC (Tailscale)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Press Ctrl+C to stop"
wait