sudo systemctl stop zed_x_daemon
sudo systemctl stop nvargus-daemon
sudo pkill -9 -f zed
sudo pkill -9 -f argus
sudo modprobe -r sl_zedx
sleep 5
sudo modprobe sl_zedx
sudo systemctl start nvargus-daemon
sudo systemctl start zed_x_daemon 