#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/Vector3Stamped.h"

#include <Eigen/Dense>
#include <Eigen/Core>
//#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <cmath>

using namespace Eigen;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eigen_transform");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

/*    Eigen::Vector3d world_point(1, 0, 0);
    Eigen::Vector3d tmp_point = world_point;
    Eigen::Vector3d euler_angle_deg(0, 0, 90);
    double s = 0.667;
    int count = 0;
    std::cout << "Current point position " << world_point.transpose() << std::endl << std::endl;*/

    while (ros::ok())
    {
//        Eigen::Vector3d euler_angle_rad = deg2rad(euler_angle_deg);
//        std::cout << euler_angle_rad.transpose() << std::endl;

//        Eigen::Quaterniond Q_Total = Euler2Quaternion(euler_angle_rad);
//        std::cout << Q_Total.coeffs().transpose() << std::endl;

        Eigen::Quaterniond Q_imu;
        Eigen::Quaterniond Q_t;
        Eigen::Quaterniond Q_opti;
        
        Q_imu.x() = -0.00137816342145;
        Q_imu.y() = 0.00740300075689;
        Q_imu.z() = 0.30623820049;
        Q_imu.w() = -0.951925137985;
        Q_opti.x() = -0.0159944482148;
        Q_opti.y() = -0.00824303552508;
        Q_opti.z() = 0.0249628871679;
        Q_opti.w() = -0.999526500702;
/*
        Q_imu.x() = 0.0270716100931;
        Q_imu.y() = -0.0137059027329;
        Q_imu.z() = -0.348701328039;
        Q_imu.w() = 0.93674248457;
        Q_opti.x() = -0.0185185689479;
        Q_opti.y() = 0.00123811350204;
        Q_opti.z() = -0.721371889114;
        Q_opti.w() = -0.692299246788;*/
/*
        Q_imu.x() = 0.00947535596788;   
        Q_imu.y() = -0.00598858809099;  
        Q_imu.z() = -0.436086565256;    
        Q_imu.w() = 0.899834930897;     
        Q_opti.x() = -0.0202923081815;  
        Q_opti.y() = 0.0119954803959;   
        Q_opti.z() = -0.694919586182;   
        Q_opti.w() = -0.718701064587;    */
        Q_t = Q_imu.inverse();
        //std::cout << Q_tmp.coeffs() << std::endl;
        //std::cout << Q_t.coeffs() << std::endl;
        std::cout << (Q_t*Q_opti).coeffs()  << std::endl;
        //std::cout << (Q_opti*Q_imu*Q_opti.conjugate()).coeffs()  << std::endl;

//        Eigen::Vector3d euler_tmp_rad = Quaternion2Euler(Q_tmp);
//        Eigen::Vector3d euler_tmp_deg = rad2deg(euler_tmp_rad);
//        std::cout << euler_tmp_deg.transpose() << std::endl;

        // Apply rotation to point with quaternion
        
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}