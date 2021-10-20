
#include "manta_dxl/dxl_func.h"
#include <ros/ros.h>

#include <ros/package.h>

#include <chrono>
#include <sys/time.h>
#include <ctime>
#include <algorithm>
#include <yaml-cpp/yaml.h>

DxlControl::DxlControl()
{

  fin_check_need = false;
  is_running = false;

  dxl_ids[0] = 1;
  dxl_ids[1] = 9;
  dxl_ids[2] = 10;
  dxl_ids[3] = 11;
  dxl_ids[4] = 12;

  for (int i = 0; i < DXL_CNT; i++)
  {

    dxlconfig[i].id = dxl_ids[i];


    dxlconfig[i].min = ctrl_item_data(dxl_model[i], "min_pos");
    dxlconfig[i].cen = ctrl_item_data(dxl_model[i], "cen_pos");
    dxlconfig[i].zero = ctrl_item_data(dxl_model[i], "zero_pos");
    dxlconfig[i].max = ctrl_item_data(dxl_model[i], "max_pos");

    if (i == 2)
    {
      dxlconfig[i].cen = 0;
    }
    // ROS_INFO("%d : %d %d ",dxlconfig[i].id,dxlconfig[i].add_torque_enable, dxlconfig[i].len_torque_enable);
    // ROS_INFO("%d : %d %d ",dxlconfig[i].id,dxlconfig[i].add_curr_position, dxlconfig[i].len_curr_position);
    // ROS_INFO("%d : %d %d ",dxlconfig[i].id,dxlconfig[i].add_goal_position, dxlconfig[i].len_goal_position);
  }
  time_checking = 10;
}

DxlControl::~DxlControl()
{
}

int DxlControl::initDxlModel(dynamixel::PacketHandler *pkh, dynamixel::PortHandler *pth)
{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  for (int i = 0; i < DXL_CNT; i++)
  {
    // int8_t temp_op = 4;
    // if(i==1 || i ==3)
    // {
    //   temp_op =3;
    // }
    dxl_comm_result = pkh->write1ByteTxRx(pth, dxlconfig[i].id, ADD_OPERATION_MODE, 4, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {

      ROS_ERROR("Failed to change oper motion for Dynamixel ID: %d", dxlconfig[i].id);
      return -1;
    }
  }
  check_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  return dxl_comm_result;
}

int DxlControl::bulkReadPosition(dynamixel::PacketHandler *pkh, dynamixel::PortHandler *pth, dynamixel::GroupBulkRead gbr, dynamixel::GroupBulkRead gbr_t)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;

  for (int i = DXL_CNT_T; i < DXL_CNT; i++)
  {
    dxl_addparam_result = gbr.addParam(dxlconfig[i].id, ADD_PRES_POSITION, LEN_PRES_POSITION);

    if (dxl_addparam_result != true)
    {
      ROS_ERROR("Failed to addparam to groupBulkRead for Dynamixel ID: %d", dxlconfig[i].id);
      return 0;
    }
  }

  dxl_comm_result = gbr.txRxPacket();
  if (dxl_comm_result == COMM_SUCCESS)
  {
    for (int i = DXL_CNT_T; i < DXL_CNT; i++)
    {
      dxlconfig[i].curr_pose = gbr.getData(dxlconfig[i].id, ADD_PRES_POSITION,LEN_PRES_POSITION);
    }
  }
  else
  {
    ROS_ERROR("Failed to get position! Result: %d", dxl_comm_result);
    gbr.clearParam();
    return 0;
  }
  gbr.clearParam();

  for (int i = 0; i < DXL_CNT_T; i++)
  {
    dxl_addparam_result = gbr_t.addParam(dxlconfig[i].id, dxlconfig[i].add_curr_position, dxlconfig[i].len_curr_position);

    if (dxl_addparam_result != true)
    {
      ROS_ERROR("Failed to addparam to groupBulkRead for Dynamixel ID: %d", dxlconfig[i].id);
      return 0;
    }
  }

  dxl_comm_result = gbr_t.txRxPacket();
  if (dxl_comm_result == COMM_SUCCESS)
  {
    for (int i = 0; i < DXL_CNT_T; i++)
    {
      dxlconfig[i].curr_pose = gbr_t.getData(dxlconfig[i].id, dxlconfig[i].add_curr_position, dxlconfig[i].len_curr_position);
    }
  }
  else
  {
    ROS_ERROR("Failed to get position! Result: %d", dxl_comm_result);
    gbr_t.clearParam();
    return 0;
  }
  gbr_t.clearParam();

  return 1;
}

int DxlControl::bulkWritePosition(dynamixel::PacketHandler *pkh, dynamixel::PortHandler *pth, dynamixel::GroupBulkWrite gbw, dynamixel::GroupBulkWrite gbw_t)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;

  uint8_t param_goal_pos_4[4];

  for (int i = DXL_CNT_T; i < DXL_CNT; i++)
  {

    uint32_t temp_pos = dxlconfig[i].goal_pose;
    // temp_pos = twos_complement(temp_pos);
    uint32_t utemp_pos = (unsigned int)temp_pos; // Convert int32 -> uint32

    param_goal_pos_4[0] = DXL_LOBYTE(DXL_LOWORD(utemp_pos));
    param_goal_pos_4[1] = DXL_HIBYTE(DXL_LOWORD(utemp_pos));
    param_goal_pos_4[2] = DXL_LOBYTE(DXL_HIWORD(utemp_pos));
    param_goal_pos_4[3] = DXL_HIBYTE(DXL_HIWORD(utemp_pos));
    dxl_addparam_result = gbw.addParam(dxlconfig[i].id, dxlconfig[i].add_goal_position, dxlconfig[i].len_goal_position, param_goal_pos_4);
    if (dxl_addparam_result != true)
    {
      ROS_ERROR("Failed to addparam to groupBulkWrite for Dynamixel ID: %d", dxlconfig[i].id);
    }
  }

  dxl_comm_result = gbw.txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {

    // ROS_INFO("Failed to set position! Result: %d", dxl_comm_result);
  }

  gbw.clearParam();

  for (int i = 0; i < DXL_CNT_T; i++)
  {

    uint32_t temp_pos = dxlconfig[i].goal_pose;
    // temp_pos = twos_complement(temp_pos);
    uint32_t utemp_pos = (unsigned int)temp_pos; // Convert int32 -> uint32

    param_goal_pos_4[0] = DXL_LOBYTE(DXL_LOWORD(utemp_pos));
    param_goal_pos_4[1] = DXL_HIBYTE(DXL_LOWORD(utemp_pos));
    param_goal_pos_4[2] = DXL_LOBYTE(DXL_HIWORD(utemp_pos));
    param_goal_pos_4[3] = DXL_HIBYTE(DXL_HIWORD(utemp_pos));
    dxl_addparam_result = gbw_t.addParam(dxlconfig[i].id, dxlconfig[i].add_goal_position, dxlconfig[i].len_goal_position, param_goal_pos_4);
    if (dxl_addparam_result != true)
    {
      ROS_ERROR("Failed to addparam to groupBulkWrite for Dynamixel ID: %d", dxlconfig[i].id);
    }
  }

  dxl_comm_result = gbw_t.txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {

    // ROS_INFO("Failed to set position! Result: %d", dxl_comm_result);
  }

  gbw_t.clearParam();
  // dxl_comm_result = pkh->write4ByteTxRx(pth, dxlconfig[0].id,dxlconfig[0].add_goal_position,  dxlconfig[0].goal_pose , &dxl_error);

  return 1;
}

int DxlControl::motorTorque(bool onoff, dynamixel::PacketHandler *pkh, dynamixel::PortHandler *pth)
{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  for (int i = 0; i < DXL_CNT; i++)
  {
    dxlconfig[i].curr_torque = onoff;

    dxl_comm_result = pkh->write1ByteTxRx(pth, dxlconfig[i].id, dxlconfig[i].add_torque_enable, dxlconfig[i].curr_torque, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      ROS_ERROR("Failed to enable torque for Dynamixel ID: %d", dxlconfig[i].id);
      return -1;
    }
  }

  return dxl_comm_result;
}

void DxlControl::motorInitZero(bool check, float time_s)
{
  fin_check_need = check;
  is_running = true;

  for (int dxlnum = 0; dxlnum < DXL_CNT; dxlnum++)
  {

    dxlconfig[dxlnum].servoTrajectory.reset(dxlconfig[dxlnum].curr_pose);
    dxlconfig[dxlnum].servoTrajectory.setTargetPos(dxlconfig[dxlnum].zero, time_s);

    idle_motion.idle_motor[dxl_ids[dxlnum]].fin_motion = false;
  }
  check_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  time_checking = time_s * 1000;
}

void DxlControl::motorInitCenter(bool check, float time_s)
{
  fin_check_need = check;
  is_running = true;

  for (int dxlnum = 0; dxlnum < DXL_CNT; dxlnum++)
  {

    dxlconfig[dxlnum].servoTrajectory.reset(dxlconfig[dxlnum].curr_pose);
    dxlconfig[dxlnum].servoTrajectory.setTargetPos(dxlconfig[dxlnum].cen, time_s);

    idle_motion.idle_motor[dxl_ids[dxlnum]].fin_motion = false;
  }
  check_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  time_checking = time_s * 1000;
}
void DxlControl::updateIdle()
{

  fin_check_need = false;
  is_running = true;
  for (int dxlnum = 0; dxlnum < DXL_CNT; dxlnum++)
  {
    idle_motion.idle_motor[dxl_ids[dxlnum]].current_idx = 0;
    // ROS_INFO("%d: %d", dxl_ids[dxlnum],idle_motion.idle_motor[dxl_ids[dxlnum]].waypoint_count);

    int temp_idx = idle_motion.idle_motor[dxl_ids[dxlnum]].current_idx;
    dxlconfig[dxlnum].servoTrajectory.reset(dxlconfig[dxlnum].curr_pose);
    // ROS_INFO("%d: %d", dxl_ids[dxlnum], temp_idx);
    // ROS_INFO("%f, %d", idle_motion.idle_motor[dxl_ids[dxlnum]].waypoint[temp_idx].first, idle_motion.idle_motor[dxl_ids[dxlnum]].waypoint[temp_idx].second);
    dxlconfig[dxlnum].servoTrajectory.setTargetPos(idle_motion.idle_motor[dxl_ids[dxlnum]].waypoint[temp_idx].second, idle_motion.idle_motor[dxl_ids[dxlnum]].waypoint[temp_idx].first);

    idle_motion.idle_motor[dxl_ids[dxlnum]].fin_motion = false;
  }

  check_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  time_checking = idle_motion.motion_time * 1000;
}
int DxlControl::updateTraj()
{
  unsigned long curr_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  unsigned long dt = curr_time - check_time;
  if (dt > time_checking && dxlconfig[0].servoTrajectory.ready() && dxlconfig[1].servoTrajectory.ready() && dxlconfig[2].servoTrajectory.ready() && dxlconfig[3].servoTrajectory.ready() && dxlconfig[4].servoTrajectory.ready())
  {

    fin_check_need = false;
    is_running = false;

    for (int dxlnum = 0; dxlnum < DXL_CNT; dxlnum++)
    {
      dxlconfig[dxlnum].servoTrajectory.reset(dxlconfig[dxlnum].curr_pose);

      check_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }
  }
  else if (is_running && !fin_check_need)
  {
    for (int dxlnum = 0; dxlnum < DXL_CNT; dxlnum++)
    {
      if (!idle_motion.idle_motor[dxl_ids[dxlnum]].fin_motion)
      {
        if (dxlconfig[dxlnum].servoTrajectory.ready())
        {
          int temp_idx;
          int temp_div = (idle_motion.idle_motor[dxl_ids[dxlnum]].current_idx + 1) / idle_motion.idle_motor[dxl_ids[dxlnum]].waypoint_count;

          if (temp_div)
          {
            idle_motion.idle_motor[dxl_ids[dxlnum]].fin_motion = true;
            break;
          }

          idle_motion.idle_motor[dxl_ids[dxlnum]].current_idx = (idle_motion.idle_motor[dxl_ids[dxlnum]].current_idx + 1) % idle_motion.idle_motor[dxl_ids[dxlnum]].waypoint_count;
          temp_idx = idle_motion.idle_motor[dxl_ids[dxlnum]].current_idx;

          dxlconfig[dxlnum].servoTrajectory.reset(dxlconfig[dxlnum].curr_pose);
          dxlconfig[dxlnum].servoTrajectory.setTargetPos(idle_motion.idle_motor[dxl_ids[dxlnum]].waypoint[temp_idx].second, idle_motion.idle_motor[dxl_ids[dxlnum]].waypoint[temp_idx].first);
        }
      }
    }
  }
  return 1;
}

bool DxlControl::setGoalPose()
{

  for (int dxlnum = 0; dxlnum < DXL_CNT; dxlnum++)
  {
    dxlconfig[dxlnum].goal_pose = dxlconfig[dxlnum].servoTrajectory.update();

    // ROS_INFO("%d : %d ",dxlconfig[dxlnum].id,dxlconfig[dxlnum].curr_pose);
    // ROS_INFO("%d : %d", dxlconfig[dxlnum].id,dxlconfig[dxlnum].goal_pose );
  }
  unsigned long curr_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  unsigned long dt = curr_time - check_time;
  // ROS_INFO("%ld",dt);
  // ROS_INFO("%d | %d : %d %d ",dxlconfig[0].id,idle_motion.idle_motor[1].current_idx,dxlconfig[0].goal_pose,dxlconfig[0].curr_pose);
  // ROS_INFO("%d | %d : %d %d ", dxlconfig[1].id, idle_motion.idle_motor[9].current_idx, dxlconfig[1].goal_pose, dxlconfig[1].curr_pose);
  // ROS_INFO("%d | %d : %d %d ", dxlconfig[2].id, idle_motion.idle_motor[10].current_idx, dxlconfig[2].goal_pose, dxlconfig[2].curr_pose);
}

bool DxlControl::isRunning()
{
  return is_running;
}