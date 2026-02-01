# Python dependencies
pip3 install "numpy<2" flask flask_socketio opencv-python psutil pyyaml pillow

# ZED SDK (Jetson only)
# Download from: https://www.stereolabs.com/developers/release/
sudo ./ZED_SDK_Tegra_L4T36.4_v5.1.2.zstd.run

# ZED Link Quad Driver (Jetson only)
# Download from: https://www.stereolabs.com/developers/release/
sudo dpkg -i stereolabs-zedlink-quad_*_arm64.deb
sudo reboot