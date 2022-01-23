#include "manta_mp3/Mp3.h"
#include <yaml-cpp/yaml.h>


Mp3::Mp3()
{
	mp3_count_ = 5;
	mp3_playing_ = false;
	random_play_time_ = 30;
}

Mp3::~Mp3()
{

}

void Mp3::readMp3Yaml(std::string mp3_path)
{
	YAML::Node yaml_doc;
	try
	{
		yaml_doc = YAML::LoadFile(mp3_path.c_str());
	}
	catch(const std::exception &e)
	{
		ROS_ERROR("Fail to load yaml file");
	}
	for(int i = 0; i < mp3_count_; i++){
		std::string cmd_key = "";
		cmd_key = convertIntToString(i);
		YAML::Node mp3_data_doc = yaml_doc[cmd_key];
		mp3_data_.push_back(mp3_data_doc["name"].as<std::string>());
	}
	return;
}

void Mp3::playMp3(int mp3_num)
{
	pid_t g_play_pid = -1;
	g_play_pid = fork();
	std::string g_sound_file_path = "/home/ubuntu/catkin_ws/src/MANTA/manta_mp3/file/";
	switch(g_play_pid)
	{
	case -1:
		ROS_INFO("mp3 play error");
		break;
	case 0:
		execl("/usr/bin/mpg321", "mpg321", (g_sound_file_path + mp3_data_[mp3_num]).c_str(), "-q", (char*)0);
		break;
	}
}

bool Mp3::checkMp3(int mp3_num)
{
	if(mp3_num < mp3_count_)
		return true;
	else
		return false;
}

std::string Mp3::convertIntToString(int n)
{
	std::ostringstream ostr;
	ostr << n;
	return ostr.str();
}
