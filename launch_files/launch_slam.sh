#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
LIVOX_WS="$WORKSPACE_DIR/livox_driver2_ws"
GLIM_WS="$WORKSPACE_DIR/glim_ws"
GLIM_LOCALIZER_BUILD_LIB="$GLIM_WS/build/glim_localizer"
GLIM_LOCALIZER_INSTALL_LIB="$GLIM_WS/install/glim_localizer/lib/glim_localizer"
GLIM_CONFIG_TEMPLATE_DIR="$GLIM_WS/config_indoor"
GLIM_RUNTIME_CONFIG_DIR="$GLIM_WS/.runtime_config/control_panel_slam"

MODE="${1:-real}"

echo "SLAM Mode: $MODE"
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source "$LIVOX_WS/install/setup.bash"


# source "$WORKSPACE_DIR/fast_lio_ws/install/setup.bash"
# ros2 launch fast_lio mapping.launch.py config_path:=/home/nahyeon/box/vln-large-scale-farm/fast_lio_ws/src/FAST_LIO/config config_file:=1scout_livox.yaml

export LD_LIBRARY_PATH="$GLIM_LOCALIZER_BUILD_LIB:$GLIM_LOCALIZER_INSTALL_LIB:$GLIM_WS/install/glim_localizer/lib/glim_localizer:${LD_LIBRARY_PATH:-}"
source "$GLIM_WS/install/setup.bash"

mkdir -p "$GLIM_RUNTIME_CONFIG_DIR"
cp -f "$GLIM_CONFIG_TEMPLATE_DIR"/* "$GLIM_RUNTIME_CONFIG_DIR"/

# The control panel SLAM path does not need saved-map localization plugins.
# Keep only the extension that publishes the live localized pose used by the
# control panel and map saver. Dropping localizer_ext removes /glim_ros/localized_curr_pose,
# which leaves the HTML map blank because map_latest.png is never generated.
perl -0pi -e 's/"extension_modules":\s*\[[^\]]*\],/"extension_modules": [\n        "libmemory_monitor.so",\n        "liblocalizer_ext.so"\n    ],/s' \
  "$GLIM_RUNTIME_CONFIG_DIR/config_ros.json"

CONFIG_DIR="$GLIM_RUNTIME_CONFIG_DIR"
for required in \
  "$CONFIG_DIR/config.json" \
  "$CONFIG_DIR/config_ros.json" \
  "$CONFIG_DIR/config_sensors.json"
do
  if [ ! -f "$required" ]; then
    echo "Missing GLIM config file: $required" >&2
    exit 1
  fi
done

echo "Using GLIM config path: $CONFIG_DIR"
echo "Using GLIM template config path: $GLIM_CONFIG_TEMPLATE_DIR"
echo "Using GLIM extension search path: $GLIM_LOCALIZER_BUILD_LIB:$GLIM_LOCALIZER_INSTALL_LIB"

if ros2 pkg executables glim_converters 2>/dev/null | grep -q '^glim_converters '; then
    ros2 run glim_converters glim_converters &
else
    echo "glim_converters executable not installed; falling back to python script"
    python3 "$GLIM_WS/glim_converters.py" &
fi
CONVERTER_PID=$!
sleep 1
# ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/home/nahyeon/box/vln-large-scale-farm/glim_ws/config
ros2 run glim_ros glim_rosnode \
  --ros-args \
  -p config_path:="$CONFIG_DIR"
#   -p localizer_ext.map_path:=/home/nahyeon/box/glim_map/ 



trap "kill $CONVERTER_PID 2>/dev/null" EXIT
