#include "manta_main/Manta.h"
#include <ros/package.h>

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
void PoseCallback(const std_msgs::Int16::ConstPtr &msg)
{
  std_msgs::Int16 led;
  led = *msg;
  manta.PubLed(led);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manta_main_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  manta.init(nh);

  std::string led_path = ros::package::getPath("manta_main")+"/config/poseLed.yaml";
  manta.readLedYaml(led_path);
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