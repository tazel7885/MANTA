#include <stdio.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <std_msgs/Int16.h>

#include "../include/manta_led/Led.hpp"


int pin_nums[3] = {21, 20, 16}; //B, R, G
int pwm_initial = 0;
int pwm_range = 1000;

float hz=0.1;


using namespace LED;

int size_pin_nums = sizeof(pin_nums)/sizeof(*pin_nums);
LedManager led_manager(pin_nums, size_pin_nums, pwm_initial ,pwm_range);

int id_msg;

// LED Input Callback
void LEDCallback(const std_msgs::Int16::ConstPtr &msg)
{
  //ROS_INFO("call: %d",msg->data);
  led_manager.SetTargetColor(hz, id_msg, msg->data);
  id_msg = msg->data;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "manta_led_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1.0/hz);

  ros::Subscriber LED_sub = nh.subscribe("/manta/led", 10, LEDCallback);

  //std::string id_file = "/home/ubuntu/catkin_ws/src/EDIE_Parasol/EDIE_ModeChanger/setting/EDIE_ID.txt";
  std::string id_file = ros::package::getPath("manta_led")+"/led_data/ID.txt";
  std::string color_file = ros::package::getPath("manta_led")+"/led_data/led_color.txt";
  
  id_msg=led_manager.ReadID(id_file);
  led_manager.ReadLedColor(color_file,pwm_range);

  while(ros::ok())
  {
    //led_manager.ReadID(id_file);
    led_manager.SetID(id_msg);
    led_manager.SetLedColor();
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  led_manager.StopLed();
  loop_rate.sleep();
}
