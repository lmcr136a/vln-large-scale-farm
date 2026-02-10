#!/bin/bash

# NMEA driver 실행
ros2 run nmea_navsat_driver nmea_serial_driver \
  --ros-args \
  -p port:=/tmp/gps_filtered \
  -p baud:=9600 \
  -p frame_id:=gps \
  -r fix:=/gps/fix &

# 1초 대기
sleep 1

# GPS to Odom 실행
python3 gps_to_odom.py &

wait
