export CYCLONEDDS_URI=file:/home/nahyeon/box/vln-large-scale-farm/config/cyclonedds_jetson.xml
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
source "$WORKSPACE_DIR/ros2_ws/xsens_ws/install/setup.bash"
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py

# python3 $WORKSPACE_DIR/ros2_ws/xsens_ws/dummy_imu.py