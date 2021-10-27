#include "manta_main/Manta.h"

#include <yaml-cpp/yaml.h>

Manta::Manta()
{
}

Manta::~Manta()
{
}

void Manta::init(ros::NodeHandle &root_nh)
{
	motion_pub = root_nh.advertise<diagnostic_msgs::KeyValue>("/manta/motion_start", 1000);
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
	return;
}

void Manta::PubMotion(diagnostic_msgs::KeyValue motion)
{
	motion_pub.publish(motion);
	return;
}

void Manta::PubLed(std_msgs::Int16 led)
{
	led_pub.publish(led);
	return;
}