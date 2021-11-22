
#include "manta_dxl/Parser.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <utility>

#include <ros/ros.h>
#include <ros/package.h>

#include <yaml-cpp/yaml.h>

static inline std::string &ltrim(std::string &s)
{
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
  return s;
}
static inline std::string &rtrim(std::string &s)
{
  s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
  return s;
}
static inline std::string &trim(std::string &s)
{
  return ltrim(rtrim(s));
}

static inline std::vector<std::string> split(const std::string &text, char sep)
{
  std::vector<std::string> tokens;
  std::size_t start = 0, end = 0;

  while ((end = text.find(sep, start)) != (std::string::npos))
  {
    tokens.push_back(text.substr(start, end - start));
    trim(tokens.back());
    start = end + 1;
  }
  tokens.push_back(text.substr(start));
  trim(tokens.back());

  return tokens;
}

DXLMotor::DXLMotor(std::string motor_file_path)
{
  std::ifstream file(motor_file_path.c_str());
  if (file.is_open())
  {
    std::string session = "";
    std::string input_str;
    while (!file.eof())
    {
      std::getline(file, input_str);

      // remove comment ( # )
      std::size_t pos = input_str.find("#");
      if (pos != std::string::npos)
        input_str = input_str.substr(0, pos);

      // trim
      input_str = trim(input_str);
      // find session
      if (!input_str.compare(0, 1, "[") && !input_str.compare(input_str.size() - 1, 1, "]"))
      {
        input_str = input_str.substr(1, input_str.size() - 2);
        std::transform(input_str.begin(), input_str.end(), input_str.begin(), ::tolower);
        session = trim(input_str);
        continue;
      }
      // ROS_INFO("%s",session.c_str());
      // ROS_INFO("%s",input_str.c_str());
      // ROS_INFO("==========");

      if (session == SESSION_PORT_INFO)
      {
        std::vector<std::string> tokens = split(input_str, '|');
        if (tokens.size() != 2)
          continue;

        std::cout << tokens[0] << " added. (baudrate: " << tokens[1] << ")" << std::endl;

        port_handler = dynamixel::PortHandler::getPortHandler(tokens[0].c_str());

        port_handler->setBaudRate(std::atoi(tokens[1].c_str()));
      }

      else if (session == SESSION_DEVICE_INFO)
      {
        std::vector<std::string> tokens = split(input_str, '|');
        if (tokens.size() != 6)
          continue;

        int id = std::atoi(tokens[0].c_str());

        device temp_device;
        temp_device.model = tokens[1].c_str();
        temp_device.device_loc = tokens[2].c_str();
        temp_device.device_name = tokens[3].c_str();
        temp_device.min = std::atoi(tokens[4].c_str());
        temp_device.max = std::atoi(tokens[5].c_str());
        dxls_[id] = temp_device;

        dxl_locations.insert(temp_device.device_loc);
      }
    }

    file.close();
  }
  else
  {
    std::cout << "Unable to open file : " + motor_file_path << std::endl;
  }

  pkt_handler = dynamixel::PacketHandler::getPacketHandler(2.0);

  port_to_bulk_read_ = new dynamixel::GroupBulkRead(port_handler, pkt_handler);

  port_to_sync_write_ = new dynamixel::GroupSyncWrite(port_handler, pkt_handler, ADD_GOAL_POSITION, LEN_GOAL_POSITION);

  port_to_bulk_read_->clearParam();
  port_to_sync_write_->clearParam();
}

DXLMotor::~DXLMotor()
{
}

int DXLMotor::InitializeDXL()
{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  for (auto &it : dxls_)
  {
    dxl_comm_result = pkt_handler->write1ByteTxRx(port_handler, it.first, ADD_OPERATION_MODE, 3, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {

      ROS_ERROR("Failed to change oper motion for Dynamixel ID: %d", it.first);
      return -1;
    }

    dxl_comm_result = port_to_bulk_read_->addParam(it.first, ADD_PRES_POSITION, LEN_PRES_POSITION);
    if (dxl_comm_result != true)
    {
      ROS_ERROR("Failed to addparam to groupBulkRead for Dynamixel ID: %d", it.first);
      return -1;
    }

    uint32_t read_data = 0;
    uint8_t sync_write_data[4];

    read_data = port_to_bulk_read_->getData(it.first, ADD_PRES_POSITION, LEN_PRES_POSITION);

    sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(read_data));
    sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(read_data));
    sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(read_data));
    sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(read_data));

    dxl_comm_result = port_to_sync_write_->addParam(it.first, sync_write_data);
    if (dxl_comm_result != true)
    {
      ROS_ERROR("Failed to addparam to groupBulkRead for Dynamixel ID: %d", it.first);
      return -1;
    }
  }

  return 1;
}
int DXLMotor::SetTorque(bool onoff)
{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  for (auto &it : dxls_)
  {
    dxl_comm_result = pkt_handler->write1ByteTxRx(port_handler, it.first, ADD_TORQUE, onoff, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {

      ROS_ERROR("Failed to change oper motion for Dynamixel ID: %d", it.first);
      return -1;
    }
  }
  return 1;
}

void DXLMotor::BulkReadMotor()
{

  int result = port_to_bulk_read_->txRxPacket();
  for (auto &it : dxls_)
  {
    uint32_t data = 0;
    if (port_to_bulk_read_->isAvailable(it.first, ADD_PRES_POSITION, LEN_PRES_POSITION))
    {
      current_position[it.first] = port_to_bulk_read_->getData(it.first, ADD_PRES_POSITION, LEN_PRES_POSITION);
    }
    // ROS_INFO("curr %d : %d", it.first, current_position[it.first]);
  }
}

void DXLMotor::BulkWriteMotor()
{
  for (auto &it : dxls_)
  {
    if(goal_position[it.first]<it.second.min)
      goal_position[it.first]=it.second.min;
    else if(goal_position[it.first]>it.second.max)
      goal_position[it.first]=it.second.max;

    uint32_t pos_data = goal_position[it.first];

    uint8_t sync_write_data[4] = {0};
    sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
    sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
    sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
    sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

    port_to_sync_write_->changeParam(it.first, sync_write_data);
  }

  port_to_sync_write_->txPacket();
}

bool DXLMotor::checkMotionLive()
{
  for(auto &it : dxls_)
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result = pkt_handler->read1ByteTxRx(port_handler, it.first, ADD_HARDWARE_ERROR, &dxl_error);
    ROS_INFO("error: %d", dxl_comm_result);
    if(dxl_comm_result){
      return true;
    } 
    //ROS_INFO("dxl_error: %d", dxl_error);
  }
  return false;
}
DXLMotion::DXLMotion()
{
}
DXLMotion::~DXLMotion()
{
}

void DXLMotion::init(ros::NodeHandle &root_nh)
{
  motionDone_pub = root_nh.advertise<diagnostic_msgs::KeyValue>("/manta/motion/done", 10);
}

void DXLMotion::SetupMotion(std::string motion_file_path, std::set<std::string> dxl_location)
{
  dxl_sessions = dxl_location;
  std::ifstream file(motion_file_path.c_str());
  if (file.is_open())
  {
    std::string session = "";
    std::string input_str;
    while (!file.eof())
    {
      std::getline(file, input_str);

      // remove comment ( # )
      std::size_t pos = input_str.find("#");
      if (pos != std::string::npos)
        input_str = input_str.substr(0, pos);

      // trim
      input_str = trim(input_str);
      // find session
      if (!input_str.compare(0, 1, "[") && !input_str.compare(input_str.size() - 1, 1, "]"))
      {
        input_str = input_str.substr(1, input_str.size() - 2);
        std::transform(input_str.begin(), input_str.end(), input_str.begin(), ::tolower);
        session = trim(input_str);
        continue;
      }
      // ROS_INFO("%s",session.c_str());
      // ROS_INFO("%s",input_str.c_str());
      // ROS_INFO("==========");

      for (auto &it : dxl_location)
      {

        group2currmotion[it.c_str()] = 0;
        if (session == it.c_str())
        {
          std::vector<std::string> tokens = split(input_str, '|');
          if (tokens.size() != 2)
            continue;

          int motion_id = std::atoi(tokens[0].c_str());
          std::string motion_name = tokens[1].c_str();
          std::string location = "/home/ubuntu/catkin_ws/src/MANTA/manta_dxl/config/";
          group_motions[session].motions[motion_id] = readYamlMotion(session, location + session + "/" + motion_name + ".yaml");
        }
      }
    }

    file.close();
  }
  else
  {
    std::cout << "Unable to open file : " + motion_file_path << std::endl;
  }
}
std::string DXLMotion::convertIntToString(int n)
{
  std::ostringstream ostr;
  ostr << n;
  return ostr.str();
}

void DXLMotion::SetupDXL(std::map<int, device> dxl_info)
{
  for (auto &it : dxl_info)
  {
    dxl_ids[it.first] = it.second.device_loc;
    motor2currindex[it.first] = 0;
    motor2trajectory[it.first].reset(0);
    motor2finmotion[it.first] = true;
  }
  return;
}

void DXLMotion::Changemotion(std::string group, int motion,std::map<int, int32_t> current_position)
{
  group2currmotion[group] = motion;

  for (auto &it : dxl_ids)
  {
    if (it.second == group)
    {
      motor2currindex[it.first] = 0;

      motor2finmotion[it.first] = false;
      
      motor2trajectory[it.first].reset(current_position[it.first]);
      int way_p = group_motions[it.second].motions[group2currmotion[it.second]].motor_id2waypoints[it.first].waypoints[motor2currindex[it.first]].second;
      float way_t = group_motions[it.second].motions[group2currmotion[it.second]].motor_id2waypoints[it.first].waypoints[motor2currindex[it.first]].first;
      
      motor2trajectory[it.first].setTargetPos(way_p,way_t);
 

    }
  }
  return;
}


void DXLMotion::UpdateMotorTarget(std::map<int, int32_t> &goal_position,std::map<int, int32_t> current_position, ros::NodeHandle &root_nh)
{

  for(auto &it : dxl_sessions)
  {
    if(CheckMotionFin(it.c_str()))
    {
      motionDone.key = it;
      motionDone.value = group2currmotion[it.c_str()];
      motionDone_pub.publish(motionDone);
      group2currmotion[it.c_str()] = 0;
      Changemotion(it.c_str(),group2currmotion[it.c_str()],current_position);
    }
  }

  for (auto &it : dxl_ids)
  {
    if (motor2trajectory[it.first].ready())
    {
      int max_idx = group_motions[it.second].motions[group2currmotion[it.second]].motor_id2waypoints[it.first].waypoint_count;
      int temp_div = (motor2currindex[it.first] + 1) / max_idx;

      if (temp_div)
      {
        motor2finmotion[it.first] = true;

        motor2trajectory[it.first].reset(current_position[it.first]);
        // motor2trajectory[it.first].setTargetPos(current_position[it.first], 10);
        continue;
      }
      else
      {
        motor2currindex[it.first] = (motor2currindex[it.first] + 1) % max_idx;
        int way_p = group_motions[it.second].motions[group2currmotion[it.second]].motor_id2waypoints[it.first].waypoints[motor2currindex[it.first]].second;
        float way_t = group_motions[it.second].motions[group2currmotion[it.second]].motor_id2waypoints[it.first].waypoints[motor2currindex[it.first]].first;
      
        motor2trajectory[it.first].reset(current_position[it.first]);
        motor2trajectory[it.first].setTargetPos(way_p,way_t);
      }
    }
    goal_position[it.first] = motor2trajectory[it.first].update();
  }
  return;
}
bool DXLMotion::CheckMotionFin(std::string session)
{ 
  bool motionfin = false;
  for (auto &it : dxl_ids)
  {
    if(it.second == session)
    {
      motionfin = motor2finmotion[it.first];
    }
  }
  return motionfin;
}
void DXLMotion::ResetMotorCurrent(std::map<int, int32_t> current_position)
{
  for (auto &it : dxl_ids)
  {
    motor2trajectory[it.first].reset(current_position[it.first]);
  }
  return;
}

motion_data DXLMotion::readYamlMotion(std::string session, std::string motion_yaml_file)
{
  motion_data temp_motion_data;

  YAML::Node yaml_doc;
  try
  {
    yaml_doc = YAML::LoadFile(motion_yaml_file.c_str());
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Fail to load yaml file : %s", motion_yaml_file.c_str());
  }

  std::string cmd_key = "";
  float max_motor_time = 0;
  for (auto &it : dxl_ids)
  {
    if (it.second != session)
      continue;
    // cout << *iter << endl;

    motor_waypoints temp_motor_waypoints;

    cmd_key = "motor_" + convertIntToString(it.first);
    YAML::Node motor_cmd_doc = yaml_doc[cmd_key];
    if (motor_cmd_doc == NULL)
    {
      ROS_WARN("No Data for %s in %s", cmd_key.c_str(), motion_yaml_file.c_str());
      break;
    }

    int cmd_num = 0;
    float cmd_time = 0;
    std::string cmd_key2 = "";
    while (true)
    {

      cmd_key2 = "waypoint_" + convertIntToString(cmd_num);
      YAML::Node motor_waypoint_doc = motor_cmd_doc[cmd_key2];

      if (motor_waypoint_doc == NULL)
      {
        ROS_WARN(" Fin Data %s : %s", cmd_key.c_str(), cmd_key2.c_str());
        break;
      }

      std::pair<float, int> temp_pair = std::make_pair(motor_waypoint_doc["time"].as<float>(), motor_waypoint_doc["point"].as<int>());
      cmd_time = motor_waypoint_doc["time"].as<float>();
      temp_motor_waypoints.waypoints.push_back(temp_pair);
      cmd_num++;
    }

    temp_motor_waypoints.waypoint_count = cmd_num;
    temp_motor_waypoints.waypoint_time = cmd_time;
    max_motor_time = std::max(max_motor_time, cmd_time);
    temp_motion_data.motor_id2waypoints[it.first] = temp_motor_waypoints;
    temp_motion_data.motion_time = max_motor_time;
  }
  return temp_motion_data;
}


// int Parser::countFiles()
// {
//   namespace fs = boost::filesystem;
//   fs::path Path(yaml_path_);
//   int Nb_ext = 0;
//   fs::directory_iterator end_iter;

//   for (fs::directory_iterator iter(Path); iter != end_iter; ++iter)
//   {
//     if (iter->path().extension() == format_)
//     {
//       path2num[Nb_ext]= iter->path().filename();
//       ++Nb_ext;
//     }
//   }
//   return Nb_ext;
// }
