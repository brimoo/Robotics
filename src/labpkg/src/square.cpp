#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

int main(int argc, char* argv[]){

	ros::init(argc, argv, "square");
	ros::NodeHandle nh;

	ros::Publisher pubtwist = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
	geometry_msgs::Twist twistmsg;

	ros::Rate rate(2);

	while(ros::ok()){

		twistmsg.linear.x = 5;
		twistmsg.angular.z = 0;
		for(int i = 0; i < 6; i++){
			pubtwist.publish(twistmsg);
			rate.sleep();
		}

		twistmsg.linear.x = 0;
		twistmsg.angular.z = 2 * M_PI;
		for(int i = 0; i < 6; i++){
			pubtwist.publish(twistmsg);
			rate.sleep();
		}

	}	 

}
