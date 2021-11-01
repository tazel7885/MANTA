#!/bin/bash

#/etc/X11/xinit/xinitrc.d/50-systemd-user.sh

#gpio -g mode 5 input
#gpio -g mode 6 input

#push=1
#not_push=0
#volUpBtnDown=0
#volDnBtnDown=0

  
#volUpBtn=`gpio -g read 5`
#volDnBtn=`gpio -g read 6`

# ROS
source /opt/ros/noetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash

# Openvino
#source /home/ubuntu/openvino/scripts/setupvars/test.sh
#export OpenCV_DIR=/usr/local/lib

#export DISPLAY=:0

#if [ $volDnBtn -eq $not_push -a  $volUpBtn -eq $not_push ];then
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
rosclean purge -y &
#roslaunch --wait edie_main edie_main.launch
rosrun manta_main manta_main_node
rosrun manta_dxl manta_dxl_node

#elif [ $volUpBtn -eq $push ];then
#  export ROS_MASTER_URI=http://10.42.0.2:11311
#  export ROS_HOSTNAME=10.42.0.4

#  rosrun edie_node_controller edie_node_controller.py
#elif [ $volDnBtn -eq $push ];then
 # export ROS_MASTER_URI=http://localhost:11311
 # export ROS_HOSTNAME=localhost
#fi