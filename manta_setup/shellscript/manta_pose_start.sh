#!/bin/bash

# ROS
#source /opt/ros/noetic/setup.bash
#source /home/ubuntu/catkin_ws/devel/setup.bash
#source ~/.bashrc
# Openvino
#source /home/ubuntu/openvino/scripts/setupvars/test.sh
#export OpenCV_DIR=/usr/local/lib

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
python3 /home/ubuntu/catkin_ws/src/online_pose_classification/scripts/pose_classification_node.py -d MYRIAD -i 0 -em /home/ubuntu/human-pose-estimation-0005/FP16/human-pose-estimation-0005.xml -cm /home/ubuntu/MANTA/online-human-pose-classification/pose_classification/weights/pose_clf_20211027.pkl -at ae -it 3 -t 0.1 -r


