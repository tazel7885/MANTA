#include "manta_main/manta.h"

#include <yaml-cpp/yaml.h>

Manta::Manta()
{
}

Manta::~Manta()
{
}

Manta::init(ros::NodeHandle &root_nh)
{
	motion_pub = root_nh.advertise<diagnostic_msgs::KeyValue>('/manta/motion_start', 1000);
	led_pub = root_nh.advertise<std_msgs::Int16>('/manta/led', 10);
}

Manta::readPoseYaml(std::string pose_file_path)
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
}

Manta::PubMotion(diagnostic_msgs::KeyValue motion)
{
	motion_pub.publish(motion);
}

Manta::PubLed(std_msgs::Int8 led)
{
	led_pub.publish(led);
}