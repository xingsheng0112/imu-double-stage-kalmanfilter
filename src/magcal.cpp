#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <cmath>

using namespace Eigen;
using namespace std;

double mag_max[3];
double mag_min[3];
double mag_bias[3];
double mag_scale[3];
double arc;
ros::Publisher pub;
geometry_msgs::Vector3Stamped magtest;

void Correct(const sensor_msgs::MagneticField::ConstPtr& data)
{
    Eigen::Vector3d mag(data->magnetic_field.x, data->magnetic_field.y, data->magnetic_field.z);
    mag_max[0] = 0.0000392;
    mag_max[1] = 0.0000300;
    mag_max[2] = -0.0000520;
    mag_min[0] = 0.0000315;
    mag_min[1] = 0.0000237;
    mag_min[2] = -0.0000572;
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2*65536;
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2*65536;
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2*65536;
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;
    
    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;
    mag(0) = (mag(0)*65536 - mag_bias[0]) * (avg_rad/mag_scale[0]);
    mag(1) = (mag(1)*65536 - mag_bias[1]) * (avg_rad/mag_scale[1]); 
    mag(2) = (mag(2)*65536 - mag_bias[2]) * (avg_rad/mag_scale[2]);

    std::cout << "mag_scalex: " << avg_rad/mag_scale[0]<< std::endl ;
    std::cout << "mag_scaley: " << avg_rad/mag_scale[1]<< std::endl ;
    std::cout << "mag_scalez: " << avg_rad/mag_scale[2]<< std::endl ;
    std::cout << "avg_rad: " << avg_rad << std::endl ;
    std::cout << "x: " << mag(0) << std::endl ;
    std::cout << "y: " << mag(1) << std::endl ;
    std::cout << "z: " << mag(2) << std::endl ;
    magtest.header.stamp=ros::Time::now();
    magtest.vector.x = mag(0);
    magtest.vector.y = mag(1);
    magtest.vector.z = mag(2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "EKF");
    ros::NodeHandle nh;
    ros::Subscriber imu_corr_sub;
    while (ros::ok())
    {
        pub = nh.advertise<geometry_msgs::PoseStamped>("magtest", 1);
        imu_corr_sub = nh.subscribe("/mavros/imu/mag", 1,Correct);
        ros::Rate r(400);
        while(1){
            pub.publish(magtest);
            ros::spinOnce();
            r.sleep();
        }
    }
    return 0;
}   