sudo systemctl stop zed_x_daemon
sudo pkill -9 -f zed
sudo modprobe -r sl_zedx
sleep 5
sudo modprobe sl_zedx
sudo systemctl start zed_x_daemon 