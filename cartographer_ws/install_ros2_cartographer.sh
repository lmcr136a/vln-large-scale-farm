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
# sudo apt update
# sudo apt install -y \
#     ros-humble-cartographer \
#     ros-humble-cartographer-ros \
#     ros-humble-nav2-map-server \
#     ros-humble-tf2-tools \
#     python3-colcon-common-extensions \
#     python3-catkin-pkg \
#     python3-pip \
#     python3-numpy \
#     python3-pil \
#     python3-yaml

cd "$WORKSPACE_DIR"
echo -e "${GREEN}Setting up rosdep...${RESET}"
source /opt/ros/humble/setup.bash

# rosdep init may fail if already initialized; that's fine
sudo rosdep init 2>/dev/null || true
rosdep update

echo -e "${GREEN}Installing workspace dependencies via rosdep...${RESET}"
rosdep install --from-paths src --ignore-src -r -y

echo -e "${GREEN}Building ROS2 workspace (including modified livox_ros_driver2)...${RESET}"

# Optional but helpful: clean env issues
export PATH="/usr/bin:$PATH"
export PYTHONPATH="/usr/lib/python3/dist-packages:$PYTHONPATH"

# Build everything (includes your src/livox_ros_driver2)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

mkdir -p "$WORKSPACE_DIR/output/figures"

echo -e "${GREEN}==== ROS2 Cartographer workspace built successfully! ====${RESET}"
echo -e "${YELLOW}To use the workspace, run:${RESET}"
echo -e "  source $WORKSPACE_DIR/install/setup.bash"
echo -e "${YELLOW}To confirm livox_ros_driver2 is from workspace:${RESET}"
echo -e "  ros2 pkg prefix livox_ros_driver2"