#include <ros/ros.h>
#include <diagnostic_msgs/KeyValue.h>  //Motor
#include <std_msgs/Int8.h>             //Led
#include <string>
#include <vector>

class Manta
{
public:
	Manta();
	~Manta();

	int readPoseYaml(std::string pose_file_path);
	int init(ros::NodeHandle &root_nh);
	void PubMotion(diagnostic_msgs::KeyValue motion);
	void PubLed(std_msgs::Int8 led);

private:
	ros::Publisher motion_pub;
	ros::Publisher led_pub;
	std::vector<std::pair<int, std::string> pose_data;
};