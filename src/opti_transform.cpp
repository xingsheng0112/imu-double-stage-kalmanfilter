#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/Vector3Stamped.h"

#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <cmath>

using namespace Eigen;
using namespace std;

ros::Publisher pub;
ros::Publisher pub_angle;
geometry_msgs::PoseStamped rpyGoal;
geometry_msgs::Vector3Stamped optieulerangle;

Eigen::MatrixXd pos(3, 1); 
Eigen::MatrixXd ori(4, 1);

double roll;
double pitch;
double yaw;

void Optitransform(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   
    Eigen::Vector3d pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Vector4d ori(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    Eigen::Quaterniond Q;
    Q.x() = msg->pose.orientation.x;
    Q.y() = msg->pose.orientation.y;
    Q.z() = msg->pose.orientation.z;
    Q.w() = msg->pose.orientation.w;
    roll = atan2(2 * (ori(3) * ori(0) + ori(1) * ori(2)), (1 - 2 * (ori(0) * ori(0) + ori(1) * ori(1))))/M_PI*180;
    pitch = asin(2 * (ori(3) * ori(1) - ori(0) * ori(2)))/M_PI*180;
    yaw = atan2(2 * (ori(0) * ori(1) + ori(3) * ori(2)), (1 - 2 * (ori(1) * ori(1) + ori(2) * ori(2))))/M_PI*180;

    optieulerangle.header.stamp=ros::Time::now();
    optieulerangle.vector.x = roll;
    optieulerangle.vector.y = pitch;
    optieulerangle.vector.z = yaw;

    rpyGoal.header.frame_id = "map";
    rpyGoal.header.stamp=ros::Time::now();
    rpyGoal.pose.position.x = 0.0;
    rpyGoal.pose.position.y = 0.0;
    rpyGoal.pose.position.z = 0.0;
    rpyGoal.pose.orientation.x = ori(0);
    rpyGoal.pose.orientation.y = ori(1);
    rpyGoal.pose.orientation.z = ori(2);
    rpyGoal.pose.orientation.w = ori(3);
}
int main(int argc, char** argv)
{
    //std::ofstream in;
    ros::init(argc, argv, "opti");
    ros::NodeHandle nh;
    ros::Subscriber opti_sub;
    while (ros::ok())
    {
        pub = nh.advertise<geometry_msgs::PoseStamped>("opti", 0);
        pub_angle = nh.advertise<geometry_msgs::Vector3Stamped>("optieuler", 0);
        opti_sub = nh.subscribe("/vrpn_client_node/BBQ/pose", 20,Optitransform);
        ros::Rate r(400);

        while(1){
            pub.publish(rpyGoal);
            pub_angle.publish(optieulerangle);
            ros::spinOnce();
            r.sleep();
        }
    }
    return 0;
}   
