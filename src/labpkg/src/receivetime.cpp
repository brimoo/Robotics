#include <ros/ros.h>
#include <std_msgs/Time.h>

void timeReceived(const std_msgs::Time &msg){
    ROS_INFO_STREAM("Received Time: " << msg.data);
}

int main(int argc, char* argv[]){
    
    ros::init(argc, argv, "receivetime");
    ros::NodeHandle nh;

    ros::Subscriber subtime = nh.subscribe("timetopic", 1000, &timeReceived);

    ros::spin();

    return 0;

}
