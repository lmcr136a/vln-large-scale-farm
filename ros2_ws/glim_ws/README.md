# GLIM Setup

## Desktop (Ubuntu 22.04, x86_64, CUDA 13.1)

```bash
# PPA
curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash

# Dependencies
sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev
sudo apt install -y libgtsam-points-cuda13.1-dev
sudo apt install -y ros-humble-glim-ros-cuda13.1

# CUDA PATH
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
sudo ldconfig

# Clone & build
cd ~/box/vln-large-scale-farm/ros2_ws/glim_ws/src
git clone https://github.com/koide3/glim_ext
cd ~/box/vln-large-scale-farm/glim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Jetson (Ubuntu 22.04, arm64, JetPack 6.2, CUDA 12.6)

```bash
# PPA
curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash

# Dependencies
sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev
sudo apt install -y libgtsam-points-cuda12.6-dev
sudo apt install -y ros-humble-glim-ros-cuda12.6

# CUDA PATH
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
sudo ldconfig

# Clone & build
cd ~/box/vln-large-scale-farm/ros2_ws/glim_ws/src
git clone https://github.com/koide3/glim_ros2
git clone https://github.com/koide3/glim_ext
cd ~/box/vln-large-scale-farm/glim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Run

```bash
# Terminal 1: converters (Livox CustomMsg -> PointCloud2, NavSatFix -> Pose, map QoS bridge)
source ~/box/vln-large-scale-farm/ros2_ws/glim_ws/install/setup.bash
ros2 run glim_converters glim_converters

# Terminal 2: GLIM
source ~/box/vln-large-scale-farm/ros2_ws/glim_ws/install/setup.bash
ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=~/box/vln-large-scale-farm/ros2_ws/glim_ws/config

# Terminal 3: rosbag (if needed)
ros2 bag play <bag_path> --rate 0.5
```

## Map output

- Dump saved to `/tmp/dump/` on shutdown
- Load in offline viewer: `ros2 run glim_ros offline_viewer`
