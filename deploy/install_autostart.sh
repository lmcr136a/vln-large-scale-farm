#!/bin/bash
# Install boot autostart for the VLN navigation system.
# Run once:  sudo bash deploy/install_autostart.sh
set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "Please run with sudo:  sudo bash deploy/install_autostart.sh" >&2
    exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "→ Installing passwordless CAN setup (/etc/sudoers.d/vln-can)"
install -m 0440 -o root -g root "$HERE/vln-can.sudoers" /etc/sudoers.d/vln-can
# Refuse to continue if the sudoers file is malformed (would break sudo).
visudo -cf /etc/sudoers.d/vln-can

echo "→ Installing systemd service (/etc/systemd/system/vln.service)"
install -m 0644 -o root -g root "$HERE/vln.service" /etc/systemd/system/vln.service

echo "→ Enabling service to start at boot"
systemctl daemon-reload
systemctl enable vln.service

echo ""
echo "✅ Done. The system will start on every boot."
echo "   Start now:    sudo systemctl start vln"
echo "   View it:      tmux attach -t vln"
echo "   Status/logs:  systemctl status vln   |   journalctl -u vln -b"
echo "   Stop:         sudo systemctl stop vln"
echo "   Disable:      sudo systemctl disable vln"
