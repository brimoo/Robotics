#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){

    // Initialize node
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    // Set up the transform listener
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    
    // Use a geometry message to store the transform
    geometry_msgs::TransformStamped transform;

    // Enter main loop
    while(nh.ok()){

        // Try to listen for the transform
        try{
            transform = buffer.lookupTransform("husky_beta/base_link", "husky_alpha/base_link", ros::Time(0));
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        ROS_INFO_STREAM("Transform Received: " << transform);

    }

    return 0;

}
