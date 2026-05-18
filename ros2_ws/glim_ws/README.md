# GLIM Build & Run Guide

> Set `$PROJECT_ROOT` to your project root directory before running the commands below.

---

## Desktop (Ubuntu 22.04, x86_64, CUDA 12+)

```bash
# Install via PPA (recommended — no manual build needed)
curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash
sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev
sudo apt install -y libgtsam-points-cuda13.1-dev
sudo apt install -y ros-humble-glim-ros-cuda13.1

# CUDA PATH
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc && sudo ldconfig

# Clone extension modules and build
cd $PROJECT_ROOT/ros2_ws/glim_ws/src
git clone https://github.com/koide3/glim_ext
cd $PROJECT_ROOT/ros2_ws/glim_ws
source /opt/ros/humble/setup.bash
colcon build --packages-skip glim_converters --symlink-install
source install/setup.bash
```

---

## Jetson (Ubuntu 22.04, arm64, JetPack 6.2, CUDA 12.6)

```bash
# Install via PPA
curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash
sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev
sudo apt install -y libgtsam-points-cuda12.6-dev
sudo apt install -y ros-humble-glim-ros-cuda12.6

# CUDA PATH
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc && sudo ldconfig

# Symlink iridescence headers (needed if installed manually)
sudo ln -sf /usr/local/include/iridescence/glk        /usr/local/include/glk
sudo ln -sf /usr/local/include/iridescence/implot.h   /usr/local/include/implot.h
sudo ln -sf /usr/local/include/iridescence/imgui.h    /usr/local/include/imgui.h
sudo ln -sf /usr/local/include/iridescence/imconfig.h /usr/local/include/imconfig.h

# Clone extension modules and build (viewer and imu_validator disabled)
cd $PROJECT_ROOT/ros2_ws/glim_ws/src
git clone https://github.com/koide3/glim_ext
cd $PROJECT_ROOT/ros2_ws/glim_ws
source /opt/ros/humble/setup.bash
colcon build \
  --packages-skip glim_converters \
  --cmake-args \
    -DBUILD_WITH_CUDA=ON \
    -DBUILD_WITH_VIEWER=OFF \
    -DENABLE_IMUVAL=OFF \
  --symlink-install
source install/setup.bash
```

> `glim_localizer/CMakeLists.txt` gates `init_pose_viewer` behind `BUILD_WITH_VIEWER`.
> `glim_ext/CMakeLists.txt` gates `imu_validator` behind `ENABLE_IMUVAL`.

---

## Run

```bash
# GLIM
source $PROJECT_ROOT/ros2_ws/glim_ws/install/setup.bash
ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=$PROJECT_ROOT/ros2_ws/glim_ws/config_indoor

# Replay rosbag (if needed)
ros2 bag play <bag_path> --rate 0.5
```

---

## Map Output

- Map dumped to `output_map_path` on shutdown (see `config_localizer.json`)
- Load in offline viewer: `ros2 run glim_ros offline_viewer`