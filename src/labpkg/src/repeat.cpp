#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <cmath>

int main(int argc, char** argv){

    // Initialize node
    ros::init(argc, argv, "repeat");
    ros::NodeHandle nh;

    // Set up the transform listener
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    // Set up the subscriber
    ros::Publisher pubtwist = nh.advertise<geometry_msgs::Twist>("husky_alpha/husky_velocity_controller/cmd_vel", 1000);

    // Use a geometry message to store the transform
    geometry_msgs::TransformStamped transform;

    // Use geometry message to send new twist
    geometry_msgs::Twist newTwist;

    // Enter main loop
    while(nh.ok()){

        // Reset movement
        newTwist.angular.z = 0.0;
        newTwist.linear.x = 0.0;
        pubtwist.publish(newTwist);

        // Try to listen for the transform
        try{
            transform = buffer.lookupTransform("husky_beta/base_link", "husky_alpha/base_link", ros::Time(0));
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // Get transform info to calculate velocities
        float  x1 = transform.transform.translation.x;
        float  y1 = transform.transform.translation.y;
        double t1 = transform.header.stamp.toSec();

        tf::Quaternion q1(transform.transform.rotation.x, 
                          transform.transform.rotation.y, 
                          transform.transform.rotation.z, 
                          transform.transform.rotation.w);

        tf::Matrix3x3 m1(q1);
        double roll1, pitch1, yaw1;
        m1.getRPY(roll1, pitch1, yaw1);

        float theta1 = yaw1;

        // Try to get a second transform 
        try{
            transform = buffer.lookupTransform("husky_beta/base_link", "husky_alpha/base_link", ros::Time(0));
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        
        // Same as before
        float  x2 = transform.transform.translation.x;
        float  y2 = transform.transform.translation.y;
        double t2 = transform.header.stamp.toSec();

        tf::Quaternion q2(transform.transform.rotation.x, 
                          transform.transform.rotation.y, 
                          transform.transform.rotation.z, 
                          transform.transform.rotation.w);

        tf::Matrix3x3 m2(q2);
        double roll2, pitch2, yaw2;
        m1.getRPY(roll2, pitch2, yaw2);

        float theta2 = yaw2;

        // Calculate and publish velocities
        float deltaX = x2 - x1, deltaY = y2 - y1, deltaT = t2 - t1;
        float squareX = pow(deltaX, 2), squareY = pow(deltaY, 2);
       
        newTwist.angular.z = (theta2 - theta1) / deltaT;
        newTwist.linear.x  = sqrt(squareX + squareY) / deltaT;
        pubtwist.publish(newTwist);

    }

    return 0;

}
