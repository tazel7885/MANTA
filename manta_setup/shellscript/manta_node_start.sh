#!/bin/bash

#/etc/X11/xinit/xinitrc.d/50-systemd-user.sh

# ROS
source /opt/ros/noetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash

# Openvino
#source /home/ubuntu/openvino/scripts/setupvars/test.sh
#export OpenCV_DIR=/usr/local/lib

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
rosclean purge -y 
roslaunch --wait xmanta_main manta_main.launch
