#!/bin/bash

set -e

GREEN="\033[1;32m"
YELLOW="\033[1;33m"
RED="\033[1;31m"
RESET="\033[0m"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")/cartographer_ws"

echo -e "${GREEN}==== ROS2 Cartographer Workspace Setup ====${RESET}"
echo -e "${YELLOW}Workspace: $WORKSPACE_DIR${RESET}"

echo -e "${GREEN}Installing ROS2 dependencies...${RESET}"
sudo apt update
sudo apt install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-nav2-map-server \
    ros-humble-tf2-tools \
    python3-colcon-common-extensions

echo -e "${GREEN}Installing Python dependencies...${RESET}"
python3 -m pip install --upgrade pip
python3 -m pip install numpy pillow pyyaml

cd /tmp
if [ ! -d "Livox-SDK2" ]; then
    echo -e "${GREEN}Cloning Livox-SDK2...${RESET}"
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
fi

cd Livox-SDK2
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install

cd "$WORKSPACE_DIR"

echo -e "${GREEN}Building ROS2 workspace...${RESET}"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

mkdir -p "$WORKSPACE_DIR/output/figures"

echo -e "${GREEN}==== ROS2 Cartographer workspace built successfully! ====${RESET}"
echo -e "${YELLOW}To use the workspace, run:${RESET}"
echo -e "  source $WORKSPACE_DIR/install/setup.bash"
