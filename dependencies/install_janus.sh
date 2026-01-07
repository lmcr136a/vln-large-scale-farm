#!/bin/bash

set -e

GREEN="\033[1;32m"
YELLOW="\033[1;33m"
RED="\033[1;31m"
RESET="\033[0m"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JANUS_STREAMING_DIR="$(dirname "$SCRIPT_DIR")/tools_janus"

echo -e "${GREEN}==== Janus Gateway Full Automated Installer ====${RESET}"

install_packages() {
    echo -e "${GREEN}Installing packages: $*${RESET}"
    sudo apt install -y "$@"
}

sudo apt update
sudo apt install python3-pip

install_packages libmicrohttpd-dev libjansson-dev \
    libssl-dev libsofia-sip-ua-dev libglib2.0-dev \
    libopus-dev libogg-dev libcurl4-openssl-dev liblua5.3-dev \
    libconfig-dev libsrtp2-dev v4l-utils pkg-config libtool automake \
    meson ninja-build

if dpkg -l | grep -q nvidia-l4t; then
    sudo apt install -y nvidia-l4t-gstreamer
fi
    
echo -e "${GREEN}Installing Python dependencies...${RESET}"
python3 -m pip install --upgrade pip
python3 -m pip install flask flask_socketio

cd /tmp
if [ ! -d "libnice" ]; then
    echo -e "${GREEN}Cloning libnice...${RESET}"
    git clone https://gitlab.freedesktop.org/libnice/libnice
fi

cd libnice
if [ ! -d "build" ]; then
    echo -e "${GREEN}Building libnice...${RESET}"
    meson --prefix=/usr build
    ninja -C build
    sudo ninja -C build install
fi

cd /tmp
if [ ! -d "janus-gateway" ]; then
    echo -e "${GREEN}Cloning janus-gateway...${RESET}"
    git clone https://github.com/meetecho/janus-gateway.git
fi

cd janus-gateway
echo -e "${GREEN}Running autogen.sh...${RESET}"
sh autogen.sh

echo -e "${GREEN}Configuring build...${RESET}"
./configure --prefix=/opt/janus

echo -e "${GREEN}Building janus-gateway...${RESET}"
make -j$(nproc)
sudo make install
sudo make configs

echo -e "${GREEN}Copying janus.js...${RESET}"
sudo cp /opt/janus/share/janus/html/demos/janus.js /opt/janus/share/janus/html/

if [ -f "$JANUS_STREAMING_DIR/control.html" ]; then
    echo -e "${GREEN}Copying control.html...${RESET}"
    sudo cp "$JANUS_STREAMING_DIR/control.html" /opt/janus/share/janus/html/
else
    echo -e "${YELLOW}control.html not found at $JANUS_STREAMING_DIR${RESET}"
fi

CONFIG_FILE="/opt/janus/etc/janus/janus.plugin.streaming.jcfg"
BACKUP_FILE="/opt/janus/etc/janus/janus.plugin.streaming.jcfg.bak"

echo -e "${GREEN}Updating streaming plugin config...${RESET}"
sudo cp "$CONFIG_FILE" "$BACKUP_FILE"
sudo sed -i '/rtp-sample:/,/}/d' "$CONFIG_FILE"

sudo bash -c "cat >> $CONFIG_FILE" <<'EOFCONFIG'

obsbot: {
  type = "rtp"
  id = 1
  description = "OBSBot H264 Stream"
  video = true
  audio = false
  videoport = 8004
  videopt = 100
  videocodec = "h264"
  videortpmap = "H264/90000"
  videobufferkf = false
}
EOFCONFIG

cd /opt/janus/share/janus/html/

if [ ! -f socket.io.min.js ]; then
    echo -e "${GREEN}Downloading socket.io.min.js...${RESET}"
    sudo wget https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.6.1/socket.io.min.js
fi

if [ ! -f adapter.min.js ]; then
    echo -e "${GREEN}Downloading adapter.min.js...${RESET}"
    sudo wget https://cdnjs.cloudflare.com/ajax/libs/webrtc-adapter/8.2.3/adapter.min.js
fi

echo -e "${GREEN}Installing janus binary to /usr/local/bin...${RESET}"
sudo cp /opt/janus/bin/janus /usr/local/bin/janus
sudo chmod +x /usr/local/bin/janus

echo -e "${GREEN}==== Janus installation completed! ====${RESET}"
