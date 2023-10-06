#!/bin/bash
#sudo systemctl stop sddm
#export DISPLAY=:0
#Xvfb $DISPLAY &
sudo xhost +local:
x11vnc -nopw -quiet &
xhost +SI:localuser:root
sh /home/nick/Documents/ros2-humble-docker-container/run_ros2.sh