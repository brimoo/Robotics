#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <cmath>

// Global variables for theta
double thetaRead, thetaEstimated;
bool goalCancelled = false;

void receivedReading(const sensor_msgs::Imu &msg){

    tf::Quaternion q(msg.orientation.x, 
                     msg.orientation.y, 
                     msg.orientation.z, 
                     msg.orientation.w);

    tf::Matrix3x3 m(q);

    double r, p, y;
    m.getRPY(r, p, y);

    thetaRead = y * (180 / M_PI);

}

void receivedEstimate(const geometry_msgs::PoseWithCovarianceStamped &msg){

    tf::Quaternion q(msg.pose.pose.orientation.x, 
                     msg.pose.pose.orientation.y, 
                     msg.pose.pose.orientation.z, 
                     msg.pose.pose.orientation.w);

    tf::Matrix3x3 m(q);

    double r, p, y;
    m.getRPY(r, p, y);

    thetaEstimated = y * (180 / M_PI);

}

int main(int argc, char **argv){

    ros::init(argc, argv, "detectdivergence");
    ros::NodeHandle nh;

    ros::Subscriber subReading  = nh.subscribe("/imu/data", 1000, &receivedReading);
    ros::Subscriber subEstimate = nh.subscribe("/amcl_pose", 1000, &receivedEstimate);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    while(nh.ok()){

        ros::spinOnce();

        if(goalCancelled){
            ac.cancelAllGoals();
        }

        if(abs(thetaEstimated - thetaRead) >= 10.0 && !goalCancelled){
            ROS_INFO_STREAM("Divergence Too High! Cancelling goal...");
            goalCancelled = true;
        }

    }

    return 0;

}
