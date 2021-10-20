#include <ros/ros.h>
#include <ros/package.h>

#include "manta_dxl/dxl_func.h"

#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

PortHandler *portHandler = PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler *packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

GroupBulkRead groupBulkRead(portHandler, packetHandler);
GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

GroupBulkRead groupBulkRead_Tale(portHandler, packetHandler);
GroupBulkWrite groupBulkWrite_Tale(portHandler, packetHandler);


int main(int argc, char **argv)
{

  ros::init(argc, argv, "bulk_read_write_node");
  ros::NodeHandle nh;

  ros::Rate loop_rate(125);

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (!portHandler->openPort())
  {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE))
  {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }
  DxlControl dxl_control;

  dxl_control.readConfig();

  dxl_control.initDxlModel(packetHandler, portHandler);

  dxl_control.motorTorque(1, packetHandler, portHandler);
  dxl_control.bulkReadPosition(packetHandler, portHandler, groupBulkRead, groupBulkRead_Tale);

  dxl_control.motorInitZero(true, 5);
  while (dxl_control.isRunning() && ros::ok())
  {
    dxl_control.bulkReadPosition(packetHandler, portHandler, groupBulkRead, groupBulkRead_Tale);
    
    dxl_control.updateTraj();
    dxl_control.setGoalPose();

    dxl_control.bulkWritePosition(packetHandler, portHandler, groupBulkWrite, groupBulkWrite_Tale);

    loop_rate.sleep();
    ros::spinOnce();
  }

  dxl_control.motorInitCenter(true,5);
  while (dxl_control.isRunning() && ros::ok())
  {
    dxl_control.bulkReadPosition(packetHandler, portHandler, groupBulkRead, groupBulkRead_Tale);
    
    dxl_control.updateTraj();
    dxl_control.setGoalPose();

    dxl_control.bulkWritePosition(packetHandler, portHandler, groupBulkWrite, groupBulkWrite_Tale);

    loop_rate.sleep();
    ros::spinOnce();
  }

  ROS_INFO("Start");
  // ros::ServiceServer bulk_get_item_srv = nh.advertiseService("/bulk_get_item", bulkGetItemCallback);
  // ros::Subscriber bulk_set_item_sub = nh.subscribe("/bulk_set_item", 10, bulkSetItemCallback);


  while (ros::ok())
  {
    dxl_control.bulkReadPosition(packetHandler, portHandler, groupBulkRead, groupBulkRead_Tale);

    dxl_control.updateTraj();
    dxl_control.setGoalPose();       

    dxl_control.bulkWritePosition(packetHandler, portHandler, groupBulkWrite, groupBulkWrite_Tale);

    if (!dxl_control.isRunning())
    {
      dxl_control.updateIdle();
    //   float temp_time = motion_script_data.motion_steps[current_idx].time;

    //   ROS_INFO("currend_idx: %d  | %f",current_idx,temp_time);
    //   dxl_control.updateTraj(false, temp_time, motion_script_data.motion_steps[current_idx].motor);
    //   current_idx = (current_idx + 1) % motion_script_data.motion_count;
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  dxl_control.motorTorque(0, packetHandler, portHandler);
  portHandler->closePort();
  return 0;
}