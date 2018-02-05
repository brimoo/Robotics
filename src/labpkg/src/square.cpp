#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

int main(int argc, char* argv[]){

	ros::init(argc, argv, "square");
	ros::NodeHandle nh;

	ros::Publisher pubtwist = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
	geometry_msgs::Twist twistmsg;

	ros::Rate rate(3);

	while(ros::ok()){

		twistmsg.linear.x = 5;
		twistmsg.angular.z = 0;
		for(int i = 0; i < 6; i++){
			pubtwist.publish(twistmsg);
			rate.sleep();
		}

		twistmsg.angular.z = 0;
		twistmsg.linear.x = 0;
		pubtwist.publish(twistmsg);
		rate.sleep(); 
		pubtwist.publish(twistmsg);
		rate.sleep(); 

		twistmsg.linear.x = 0;
<<<<<<< HEAD
		twistmsg.angular.z =  M_PI / 2.7;
=======
		twistmsg.angular.z =  M_PI / 3;
>>>>>>> 0495712d2cf0018613db62f6607b8e67afb53021
		for(int i = 0; i < 6; i++){
			pubtwist.publish(twistmsg);
			rate.sleep();
		}

	}	 

}
