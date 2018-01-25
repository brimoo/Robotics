#include <ros/ros.h>
#include <std_msgs/Time.h>

int main(int argc, char* argv[]){

    ros::init(argc, argv, "sendtime");
    ros::NodeHandle nh;

    ros::Publisher pubtime = nh.advertise<std_msgs::Time>("timetopic", 1000);
    std_msgs::Time timemsg;
    
    ros::Rate rate(1);

    while(ros::ok()){
        timemsg.data = ros::Time::now();
        ROS_INFO_STREAM("Sending Time:  " << timemsg.data);
        pubtime.publish(timemsg);
        rate.sleep();
    }

    return 0;

}
