#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void laserDataReceived(const sensor_msgs::LaserScan &msg){

	for(int i = 0; i < msg.ranges.size(); i++){
		ROS_INFO_STREAM(msg.ranges[i]);
	}

}

int main(int argc, char* argv[]){

	ros::init(argc, argv, "getlaser");
	ros::NodeHandle nh;

	ros::Subscriber sublaser = nh.subscribe("/scan", 1000, &laserDataReceived);

	ros::spin();

	return 0;

}
