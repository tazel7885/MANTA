#include <stdio.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "../include/manta_main/manta.h"
#include <string>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <diagnostic_msgs/KeyValue.h>


Manta manta;
void MotionCallback(diagnostic_msgs::KeyValue::ConstPtr &msg);
void PoseCallback(std_msgs::Int16::ConstPtr &msg);
