#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>

// Global variable for target pose
geometry_msgs::Pose2D targetPose;
bool newCommand = true;

void receivedPose(const geometry_msgs::Pose2D &msg){

    ROS_INFO_STREAM("Received target pose...");

    targetPose.x = msg.x;
    targetPose.y = msg.y;
    targetPose.theta = msg.theta;

    newCommand = true;

}


int main(int argc, char **argv){

    ros::init(argc, argv, "gotopose");
    ros::NodeHandle nh;
    
    ros::Subscriber subNewPose = nh.subscribe("/targetpose", 1000, &receivedPose);
    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    ROS_INFO_STREAM("Waiting for action server...");

    while(!ac.waitForServer()){}

    ROS_INFO_STREAM("Connected to action server!");

    while(nh.ok()){

        ros::spinOnce();

        if(newCommand){

            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = targetPose.x;
            goal.target_pose.pose.position.y = targetPose.y;
            goal.target_pose.pose.orientation.w = targetPose.theta;

            newCommand = false;

            ac.sendGoal(goal);

        }

    }

    return 0;

}
