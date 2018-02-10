#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <cmath>

// Global variables for target pose and current

geometry_msgs::Pose2D currentPose, targetPose;

void receivedNewPose(const geometry_msgs::Pose2D &msg){

    targetPose.x = msg.x;
    targetPose.y = msg.y;
    targetPose.theta = msg.theta;

}

void receivedTFMessage(const tf::tfMessage &msg){
   
    for(int i = 0; i < msg.transforms.size(); i++){

        if(msg.transforms[i].header.frame_id == "odom" && msg.transforms[i].child_frame_id == "base_link"){
       
            currentPose.x = msg.transforms[i].transform.translation.x;
            currentPose.y = msg.transforms[i].transform.translation.y;

            tf::Quaternion q(msg.transforms[i].transform.rotation.x,
                             msg.transforms[i].transform.rotation.y,
                             msg.transforms[i].transform.rotation.z,
                             msg.transforms[i].transform.rotation.w);

            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            currentPose.theta = yaw;

        }

    }

}

int main(int argc, char** argv){

    ros::init(argc, argv, "gotopose");
    ros::NodeHandle nh;

    ros::Publisher pubtwist = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
    ros::Subscriber subNewPose = nh.subscribe("/targetpose", 1000, &receivedNewPose);
    ros::Subscriber subTF = nh.subscribe("/tf", 1000, &receivedTFMessage);

    geometry_msgs::Twist newTwist;

    ros::Rate rate(2);

    while(ros::ok()){

        ros::spinOnce();
        
        double newHeading = atan2((targetPose.y - currentPose.y), (targetPose.x - currentPose.x)) * (180.0 / M_PI);

        double currentHeading = currentPose.theta * (180.0 / M_PI);

        double targetTheta = targetPose.theta * (180.0 / M_PI);
 
        if(abs(newHeading - currentHeading) > 5.0 && (abs(currentPose.x - targetPose.x) > 0.2 || abs(currentPose.y - targetPose.y) > 0.2)){

            newTwist.linear.x = 0;
            newTwist.angular.z = M_PI / 4;
            pubtwist.publish(newTwist);
            rate.sleep();

        }else if((abs(currentPose.x - targetPose.x) > 0.2 || abs(currentPose.y - targetPose.y) > 0.2) && abs(newHeading - currentHeading) < 5.0){

            newTwist.linear.x = 0;
            newTwist.angular.z = 0;
            pubtwist.publish(newTwist);
            rate.sleep();

            newTwist.angular.z = 0;
            newTwist.linear.x = 3;
            pubtwist.publish(newTwist);
            rate.sleep();

        }else if((abs(currentPose.x - targetPose.x) < 0.2 && abs(currentPose.y - targetPose.y) < 0.2) && abs(targetTheta - currentHeading) > 10.0){
            
            newTwist.linear.x = 0;
            newTwist.angular.z = M_PI / 4;
            pubtwist.publish(newTwist);
            rate.sleep();

        }else{

            newTwist.linear.x = 0;
            newTwist.angular.z = 0;
            pubtwist.publish(newTwist);
            rate.sleep();

        }

    }

    return 0;

}
