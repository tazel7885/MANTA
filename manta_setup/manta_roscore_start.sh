#!/bin/bash

source /opt/ros/melodic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
/opt/ros/melodic/bin/roscore