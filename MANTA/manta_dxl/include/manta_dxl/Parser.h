
#ifndef MANTA_DXL_PARSER_H
#define MANTA_DXL_PARSER_H

#include <string>
#include <vector>
#include <utility>
#include <set>
#include <ros/ros.h>
#include <ros/package.h>
#include "diagnostic_msgs/KeyValue.h"
#include "manta_dxl/trajectory.h"

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk/port_handler.h"

#define SESSION_PORT_INFO     "port info"
#define SESSION_DEVICE_INFO   "device info"

#define ADD_OPERATION_MODE 11
#define ADD_TORQUE 64
#define ADD_GOAL_POSITION 116
#define ADD_PRES_POSITION 132

#define LEN_TORQUE 1
#define LEN_GOAL_POSITION 4
#define LEN_PRES_POSITION 4

#define MAX_POSE_28 28672
#define MAX_POSE_430 1048575

typedef struct
{
  std::string model;
  std::string device_loc;
  std::string device_name;
  int min;
  int max;

} device;

class DXLMotor
{
public:
  DXLMotor(std::string motor_file_path);
  ~DXLMotor();

  int InitializeDXL();
  void BulkReadMotor();
  void BulkWriteMotor();
  int SetTorque(bool onoff);

  std::map<int, device> dxls_;   
  std::map<int, int32_t> current_position;   
  std::map<int, int32_t> goal_position;
  std::set<std::string> dxl_locations;   

private:

  dynamixel::PortHandler *port_handler;  
  dynamixel::PacketHandler *pkt_handler;

  dynamixel::GroupBulkRead *port_to_bulk_read_;
  dynamixel::GroupSyncWrite *port_to_sync_write_;

};


// each motor
typedef struct
{
  float waypoint_time;
  int waypoint_count;
  std::vector<std::pair<float, int>> waypoints;
} motor_waypoints;

// full 1 motion
typedef struct
{
  float motion_time;

  std::map<int, motor_waypoints> motor_id2waypoints;
} motion_data;

// group of tale, wing
typedef struct
{
  std::map<int,motion_data> motions;
} group_motion;

class DXLMotion
{
public:
  DXLMotion();
  ~DXLMotion();
  std::string convertIntToString(int n);

  void init(ros::NodeHandle &root_nh);
  void SetupMotion(std::string motion_file_path,std::set<std::string> dxl_location);

  motion_data readYamlMotion(std::string session,std::string motion_yaml_file);
  

  void SetupDXL(std::map<int, device>  dxl_info);

  void UpdateMotorTarget(std::map<int, int32_t> &goal_position,std::map<int, int32_t> current_position, ros::NodeHandle &root_nh);
  void ResetMotorCurrent(std::map<int, int32_t> current_position);
  void Changemotion(std::string group, int motion,std::map<int, int32_t> current_position);

  bool CheckMotionFin(std::string session);

private:

  std::map<uint8_t,std::string> dxl_ids;
  std::set<std::string> dxl_sessions;

  std::map<std::string,group_motion> group_motions;

  std::map<std::string,int> group2currmotion;
  std::map<uint8_t,int> motor2currindex;
  std::map<uint8_t, bool> motor2finmotion;
  std::map<uint8_t,Trajectory> motor2trajectory;

  ros::Publisher motionDone_pub;
  diagnostic_msgs::KeyValue motionDone;


};













// class Parser
// {
//   private:
  
//   std::string format_;
//   std::string yaml_path_;


//   std::map<std::string,int> path2num;
//   std::map<int,std::string> num2path;

//   std::map<int,motion> motions;


//   uint8_t dxl_ids[DXL_CNT];


//   public:
//     // Constructor
//     Parser();
//     // Desturctor
//     ~Parser();

//     int countFiles();

    

// };

#endif /* MANTA_DXL_PARSER_H */
