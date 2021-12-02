#include <string>
#include <vector>
#include <stdio.h>

// ros header file
#include <ros/ros.h>
#include <ros/package.h>

// ros msg header file
#include <std_msgs/Int16.h>

class Mp3
{
public:
	Mp3();
	~Mp3();

	void readMp3Yaml(std::string mp3_path);
	void playMp3(int mp3_num);
	bool checkMp3(int mp3_num);
	std::string convertIntToString(int n);

private:
	std::vector<std::string> mp3_data_;
	int mp3_count_;
	bool mp3_playing_;
	int random_play_time_;
};