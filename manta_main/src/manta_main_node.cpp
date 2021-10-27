#include <stdio.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "manta_main/manta.h"
#include <string>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <diagnostic_msgs/KeyValue.h>

Manta manta;
// Motion Done Callback
void MotionCallback(const diagnostic_msgs::KeyValue::ConstPtr &msg)
{
  diagnostic_msgs::KeyValue motion;
  motion.key = msg->key.c_str();
  motion.value = '1';
  manta.PubMotion(motion);
}

// Pose Callback
void PoseCallback(const std_mgs::Int16::ConstPtr &msg)
{
  std_mgs::Int16 led;
  led = *msg;
  manta.PubLed(led);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manta_main_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  manta.init(nh);
  //manta.readPoseYaml('path');

  ros::Subscriber Motion_sub = nh.subscribe("/manta/motion_done", 10, MotionCallback);
  ros::Subscriber Pose_sub = nh.subscribe("/manta/vision/pose", 10, PoseCallback);
  
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  loop_rate.sleep();
}
