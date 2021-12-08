#include "manta_mp3/Mp3.h"
#include <ros/package.h>

Mp3 mp3;

// Motion Done Callback
void Mp3Callback(const std_msgs::Int16::ConstPtr &msg)
{
	if(mp3.checkMp3(msg->data))
		mp3.playMp3(msg->data);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "manta_mp3_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);


	std::string mp3_path = ros::package::getPath("manta_mp3")+"/config/mp3.yaml";
	mp3.readMp3Yaml(mp3_path);

	ros::Subscriber Mp3_sub = nh.subscribe("/manta/mp3", 10, Mp3Callback);

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	loop_rate.sleep();
}