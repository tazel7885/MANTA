#include <string>
#include <vector>
#include <stdio.h>
#include <random>
 
// ros header file
#include <ros/ros.h>
#include <ros/package.h>

// ros message header file
#include <std_msgs/Int16.h> // led 
#include <diagnostic_msgs/KeyValue.h> // motor

class Manta
{
public:
	Manta();
	~Manta();

	void readPoseYaml(std::string pose_file_path);
	void readLedYaml(std::string led_file_path);
	void init(ros::NodeHandle &root_nh);
	void PubMotion(diagnostic_msgs::KeyValue motion);
	void PubLed(std_msgs::Int16 led);
	void PubMp3();
	void PubCheck();
	std::string convertIntToString(int n);

private:
	ros::Publisher motion_pub;
	ros::Publisher led_pub;
	ros::Publisher mp3_pub;
	ros::Publisher check_pub;
	std::vector<std::pair<std::string, int>> pose_data;
	std::vector<std::pair<std::string, int>> led_data;
	int pose_num;
	int mp3_num;
	int count;
	int count_time; // target_sec / control_cycle
	
};
