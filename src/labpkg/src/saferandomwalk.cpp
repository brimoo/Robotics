#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <cstdlib>
#include <cmath>

// Global variables for target pose and nearest laser reading;
float nearestObstacle;
bool newGoal = true;
geometry_msgs::Pose2D targetPose;

void receivedReading(const sensor_msgs::LaserScan &msg){
    
    float min = msg.ranges[0];

    for(int i = 1; i < msg.ranges.size(); i++){
        if(msg.ranges[i] < min)
            min = msg.ranges[i];
    }

    nearestObstacle = min;

}

float genRandom(float a, float b){return ((b - a) * ((float) rand() / RAND_MAX)) + a;}

void randomGoal(){

    targetPose.x = genRandom(-5.0, 5.0);
    targetPose.y = genRandom(-5.0, 5.0);
    targetPose.theta = genRandom(0.0, 2 * M_PI);

    newGoal = true;

}

int main(int argc, char **argv){

    ros::init(argc, argv, "gotopose");
    ros::NodeHandle nh;
    
    ros::Subscriber subReading = nh.subscribe("/scan", 1000, &receivedReading);
    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    ROS_INFO_STREAM("Waiting for action server...");

    while(!ac.waitForServer()){}

    ROS_INFO_STREAM("Connected to action server!");

    srand(time(NULL));

    randomGoal();

    while(nh.ok()){

        ros::spinOnce();

        if(newGoal){

            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = targetPose.x;
            goal.target_pose.pose.position.y = targetPose.y;
            goal.target_pose.pose.orientation.w = targetPose.theta;

            newGoal = false;

            ac.sendGoal(goal);

        }

        if(nearestObstacle <= 0.2 || ac.getState().isDone()){
            ac.cancelGoal();
            randomGoal();
        }

    }

    return 0;

}
