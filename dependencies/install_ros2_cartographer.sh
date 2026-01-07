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
    python3-colcon-common-extensions \
    python3-catkin-pkg \
    python3-pip \
    python3-numpy \
    python3-pil \
    python3-yaml

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

export PATH="/usr/bin:$PATH"
export PYTHONPATH="/usr/lib/python3/dist-packages:$PYTHONPATH"

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

mkdir -p "$WORKSPACE_DIR/output/figures"

echo -e "${GREEN}==== ROS2 Cartographer workspace built successfully! ====${RESET}"
echo -e "${YELLOW}To use the workspace, run:${RESET}"
echo -e "  source $WORKSPACE_DIR/install/setup.bash"
