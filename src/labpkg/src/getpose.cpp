#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <iostream>

int main(int argc, char** argv){

    ros::init(argc, argv, "getpose");
    ros::NodeHandle nh;

    ros::Publisher pubpose = nh.advertise<geometry_msgs::Pose2D>("/targetpose", 1000);

    geometry_msgs::Pose2D newPose;

    while(ros::ok()){

        std::cout << "Enter an x value: ";
        std::cin >> newPose.x;
        std::cout << "Enter a y value: ";
        std::cin >> newPose.y;
        std::cout << "Enter a theta value: ";
        std::cin >> newPose.theta;

        pubpose.publish(newPose);

    }

    return 0;

}
