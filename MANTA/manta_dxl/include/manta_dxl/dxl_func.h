
#ifndef MANTA_DXL_DXL_FUNC
#define MANTA_DXL_DXL_FUNC

#include <map>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "manta_dxl/trajectory.h"

#include <vector>
#include <utility>

// Protocol version
#define PROTOCOL_VERSION 2.0 // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL_CNT_T 1
#define DXL_CNT 5
#define BAUDRATE 115200             // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0" // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

#define MX28 2
#define XL430 4


typedef struct dxl_config_
{

  uint8_t id;

  int32_t min;
  int32_t zero;
  int32_t cen;
  int32_t max;

  int32_t curr_pose;
  Trajectory servoTrajectory;
} dxl_config;

class DxlControl
{
private:
  
  bool fin_check_need;

  bool is_running;

  float time_checking;

	unsigned long check_time;

  // dxl
  uint8_t dxl_ids[DXL_CNT];
  dxl_config_ dxlconfig[DXL_CNT];


  motion_step idle_motion;

public:
  DxlControl();

  virtual ~DxlControl();

  int initDxlModel(dynamixel::PacketHandler *pkh, dynamixel::PortHandler *pth);

  int bulkReadPosition(dynamixel::PacketHandler *pkh, dynamixel::PortHandler *pth, dynamixel::GroupBulkRead gbr, dynamixel::GroupBulkRead gbr_t);

  int bulkWritePosition(dynamixel::PacketHandler *pkh, dynamixel::PortHandler *pth, dynamixel::GroupBulkWrite gbw, dynamixel::GroupBulkWrite gbw_t);

int motorTorque(bool onoff, dynamixel::PacketHandler *pkh, dynamixel::PortHandler *pth);

void motorInitZero(bool check,float time_s);
void motorInitCenter(bool check,float time_s);

// int updateTraj(bool check,float time_s, const std::vector<int> &vect);

int updateTraj();
void updateIdle();
bool setGoalPose();

bool isRunning();
bool readConfig();
};

#endif /* MANTA_DXL_DXL_FUNC */
