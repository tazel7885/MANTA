#include <stdio.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <std_msgs/Int8.h>
#include <diagnostic_msgs/KeyValue.h>
//#include "../include/manta_main/manta_main_node.hpp"


diagnostic_msgs::KeyValue motion;
ros::Publisher Motion_pub = nh.advertise<diagnostic_msgs::KeyValue>('/manta/motion_start', 1000);
// Motion Done Callback
void MotionCallback(const diagnostic_msgs::KeyValue &msg)
{
  //ROS_INFO("call: %d",msg->data);
  motion.key = msg->key;
  motion.value = 1;
  Motion_pub.publish(motion);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "manta_main_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ros::Subscriber Motion_sub = nh.subscribe("/manta/motion_done", 10, MotionCallback);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  loop_rate.sleep();
}
