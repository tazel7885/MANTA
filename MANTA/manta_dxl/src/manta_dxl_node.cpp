#include <ros/ros.h>
#include <ros/package.h>
#include "manta_dxl/Parser.h"
  
#include "diagnostic_msgs/KeyValue.h"

DXLMotor dxlmotors("/home/ubuntu/catkin_ws/src/MANTA/manta_dxl/config/manta.motor");
DXLMotion dxlmotion;

void decisionCallback(const diagnostic_msgs::KeyValue::ConstPtr& msg)
{
  ROS_INFO("[%s] : %s", msg->key.c_str(), msg->value.c_str());
  std::string group=msg->key.c_str();
  int motion=std::atoi(msg->value.c_str());
  dxlmotion.Changemotion(group, motion, dxlmotors.current_position);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "manta_dxl_node");
  ros::NodeHandle nh;

  ros::Rate loop_rate(125);

  dxlmotors.InitializeDXL();

  dxlmotion.init(nh);  
  dxlmotion.SetupDXL(dxlmotors.dxls_);
  dxlmotion.SetupMotion("/home/ubuntu/catkin_ws/src/MANTA/manta_dxl/config/manta.motion",dxlmotors.dxl_locations);

  dxlmotors.SetTorque(true);
  dxlmotors.BulkReadMotor();
  dxlmotion.ResetMotorCurrent(dxlmotors.current_position);
  // UpdateMotorTarget(std::map<int, int32_t> &goal_position,std::map<int, int32_t> current_position);


  ros::Subscriber sub = nh.subscribe("/manta/motion_start", 1000, decisionCallback);

  while (ros::ok())
  {
    dxlmotors.BulkReadMotor();
    dxlmotion.UpdateMotorTarget(dxlmotors.goal_position,dxlmotors.current_position, nh);

    dxlmotors.BulkWriteMotor();
 
    loop_rate.sleep();
    ros::spinOnce();
  }
  dxlmotors.SetTorque(false);

  return 0;
}