#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

void receivedNewPose(const geometry_msgs::Pose2D &msg){
    return;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "gotopose");
    ros::NodeHandle nh;

    ros::Publisher pubtwist = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
    ros::Subscriber subNewPose = nh.subscribe("/targetpose", 1000, &receivedNewPose);

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    geometry_msgs::Twist newTwist;
    geometry_msgs::TransformStamped transformStamped;

    while(nh.ok()){

        ros::spinOnce();
        
        try{
            transformStamped = buffer.lookupTransform("base_link", "odom", ros::Time(0));
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

    }

}
