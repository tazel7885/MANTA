#include "manta_main/Manta.h"
#include <yaml-cpp/yaml.h>

Manta::Manta()
{
	pose_num = 8;
}

Manta::~Manta()
{
}

void Manta::init(ros::NodeHandle &root_nh)
{
	motion_pub = root_nh.advertise<diagnostic_msgs::KeyValue>("/manta/motion/start", 1000);
	led_pub = root_nh.advertise<std_msgs::Int16>("/manta/led", 10);
	return;
}

void Manta::readPoseYaml(std::string pose_file_path)
{
	YAML::Node yaml_doc;
	try
	{
		yaml_doc = YAML::LoadFile(pose_file_path.c_str());
	}
	catch(const std::exception &e)
	{
		ROS_ERROR("Fail to load yaml file");
	}
	for(int i = 0; i < pose_num; i++){
		std::string cmd_key = "";
		cmd_key = convertIntToString(i);
		YAML::Node Pose_data_doc = yaml_doc[cmd_key];
		pose_data.push_back(std::make_pair(Pose_data_doc["name"].as<std::string>(), i));
	}
	return;
}

void Manta::readLedYaml(std::string led_file_path)
{
	YAML::Node yaml_doc;
	try
	{
		yaml_doc = YAML::LoadFile(led_file_path.c_str());
	}
	catch(const std::exception &e)
	{
		ROS_ERROR("Fail to load yaml file");
	}
	for(int i = 0; i < pose_num; i++){
		std::string cmd_key = "";
		cmd_key = convertIntToString(i);
		YAML::Node Led_data_doc = yaml_doc[cmd_key];
		led_data.push_back(std::make_pair(Led_data_doc["name"].as<std::string>(), Led_data_doc["led"].as<int>()));
	}
	return;
}

void Manta::PubMotion(diagnostic_msgs::KeyValue motion)
{
	motion_pub.publish(motion);
	return;
}

void Manta::PubLed(std_msgs::Int16 led)
{
	for(int i = 0; i < pose_num; i++)
	{
		if(pose_data[led.data].first == led_data[i].first)
		{
			std_msgs::Int16 data;
			data.data = led_data[i].second;
			led_pub.publish(data);
			return;
		}
	}

}

std::string Manta::convertIntToString(int n)
{
  std::ostringstream ostr;
  ostr << n;
  return ostr.str();
}