#!/bin/bash

#/etc/X11/Xinit/xinitrc.d/50-systemd-user.sh

# ROS
source /opt/ros/noetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash

roslaunch --wait manta_dxl manta_dxl.launch


