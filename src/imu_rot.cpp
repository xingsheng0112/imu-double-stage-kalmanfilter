#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <cmath>

using namespace Eigen;
using namespace std;

Eigen::MatrixXd q(4, 1);
ros::Publisher pub;
geometry_msgs::PoseStamped rpyGoal;
Eigen::Quaterniond q_imu;
Eigen::Quaterniond q_r;

void Rotate(const sensor_msgs::Imu::ConstPtr& msg)
{
    q(0) = msg->orientation.w;
    q(1) = msg->orientation.x;
    q(2) = msg->orientation.y;
    q(3) = msg->orientation.z;
    std::cout<<"q: "<<q<<endl;
    q_imu.x() = q(1);
    q_imu.y() = q(2);
    q_imu.z() = q(3);
    q_imu.w() = q(0);
    q_r.x() = -0.000390539;
    q_r.y() = 0.0156205;
    q_r.z() = 0.290278;
    q_r.w() = 0.956815;
    q_imu = (q_r * q_imu*q_r.conjugate())*q_r;
    q_imu = q_imu.normalized();
    rpyGoal.header.frame_id = "map";
    rpyGoal.pose.orientation.x = q_imu.x();
    rpyGoal.pose.orientation.y = q_imu.y();
    rpyGoal.pose.orientation.z = q_imu.z();
    rpyGoal.pose.orientation.w = q_imu.w();
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "eigen_transform");
    ros::NodeHandle nh;
    ros::Rate r(400);
    ros::Subscriber imu_rot_sub;
    while (ros::ok())
    {
        pub = nh.advertise<geometry_msgs::PoseStamped>("imu_rot", 100);
        imu_rot_sub = nh.subscribe("/mavros/imu/data", 100,Rotate);
        while(1){
            pub.publish(rpyGoal);
            ros::spinOnce();
            r.sleep();
        }
    }
    return 0;
}