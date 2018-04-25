#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Eigenvalues>
#include <ros/ros.h>
#include <cmath>

#define MAGIC 9.348

void receivedPose(const nav_msgs::Odometry &msg)
{
    Eigen::Matrix3d covMatrix;

    int k = 0;

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            while(k < 36){
                if(abs(msg.pose.covariance[k]) > 0.0001){
                    covMatrix(i, j) = msg.pose.covariance[k];
                    k++;
                    break;
                }
                k++;
            }
        }
    }

    std::complex<double> e1 = covMatrix.eigenvalues()[0];
    std::complex<double> e2 = covMatrix.eigenvalues()[1];
    std::complex<double> e3 = covMatrix.eigenvalues()[2];

    double a = sqrt(e1.real() * MAGIC);
    double b = sqrt(e2.real() * MAGIC);
    double c = sqrt(e3.real() * MAGIC);

    ROS_INFO_STREAM("Volume: " << (4.0/3.0) * M_PI * a * b * c);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "uncertainty");
    ros::NodeHandle nh;

    ros::Subscriber subPose = nh.subscribe("/odometry/filtered", 1000, &receivedPose);

    ros::spin();
    
    return 0;

}
