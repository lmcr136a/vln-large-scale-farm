#!/bin/bash

set -e

GREEN="\033[1;32m"
YELLOW="\033[1;33m"
RESET="\033[0m"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${GREEN}==== Agricultural Robot Navigation System Installation ====${RESET}"
echo -e "${YELLOW}Installation directory: $SCRIPT_DIR${RESET}"

echo -e "${YELLOW}Step 1/2: Installing Janus Gateway...${RESET}"
cd "$SCRIPT_DIR/dependencies"
./install_janus.sh

echo -e "${YELLOW}Step 2/2: Building ROS2 Cartographer workspace...${RESET}"
./install_ros2_cartographer.sh

echo -e "${GREEN}==== Installation Complete! ====${RESET}"
echo -e "${YELLOW}To start the system:${RESET}"
echo -e "  cd $SCRIPT_DIR"
echo -e "  ./scripts/start_all.sh"
