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

Eigen::MatrixXd A_tc(4, 4); 
Eigen::MatrixXd q(4, 1);
Eigen::MatrixXd Omega(4, 4);
Eigen::MatrixXd Accel(3, 1); 
Eigen::MatrixXd P(4, 4); 
Eigen::MatrixXd Q(4, 4); 
Eigen::MatrixXd I(4, 4); 

Eigen::MatrixXd H_acc(3, 4); 
Eigen::MatrixXd R_acc(4, 4);
Eigen::MatrixXd K_acc(4, 3);
Eigen::MatrixXd h_acc(3, 1);
Eigen::MatrixXd Q_acc(3, 3);
Eigen::MatrixXd q_acc(4, 1);

Eigen::MatrixXd H_mag(3, 4); 
Eigen::MatrixXd R_mag(4, 4);
Eigen::MatrixXd K_mag(4, 3);
Eigen::MatrixXd h_mag(3, 1);
Eigen::MatrixXd Q_mag(3, 3);
Eigen::MatrixXd q_mag(4, 1);

Eigen::Quaterniond q_imu;
Eigen::Quaterniond q_rotation;

double last_time = 0;
double now_time;
double dt = 0;
int flag = 1;
double mag_max[3];
double mag_min[3];
double mag_bias[3];
double mag_scale[3];
double arc;
ros::Publisher pub;
ros::Publisher pub_angle;
ros::Publisher pub_vel;

geometry_msgs::PoseStamped rpyGoal;
geometry_msgs::Vector3Stamped px4eulerangle;
geometry_msgs::Vector3Stamped angular_vel;

Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond Q)
{
    Eigen::Vector3d Euler(0, 0, 0);
    Euler.x() = atan2(2 * (Q.w() * Q.x() + Q.y() * Q.z()), (1 - 2 * (Q.x() * Q.x() + Q.y() * Q.y())))/M_PI*180;
    Euler.y() = asin(2 * (Q.w() * Q.y() - Q.z() * Q.x()))/M_PI*180;
    Euler.z() = atan2(2 * (Q.w() * Q.z() + Q.x() * Q.y()), (1 - 2 * (Q.y() * Q.y() + Q.z() * Q.z())))/M_PI*180;
    return Euler;
}   

void Predict(const sensor_msgs::Imu::ConstPtr& msg)
{   
    //Time
    if(last_time != 0)
    {
        now_time = msg->header.stamp.toSec();
        dt = now_time - last_time;
        last_time = now_time;
    }
    else
    {
        last_time = msg->header.stamp.toSec();
    }

    Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Eigen::Vector3d vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    if(flag == 1)
    {
        q(0) = msg->orientation.w;
        q(1) = msg->orientation.x;
        q(2) = msg->orientation.y;
        q(3) = msg->orientation.z;
        flag += 1;
    }
    //Predict
    Accel = acc;
    
    Omega <<    0  , -vel(0), -vel(1), -vel(2),
             vel(0),    0   ,  vel(2), -vel(1),
             vel(1), -vel(2),    0   ,  vel(0),
             vel(2),  vel(1), -vel(0),    0   ; 
    A_tc = 0.5 * Omega * dt + I;
    q = A_tc * q;
    P = A_tc * P * A_tc.transpose() + Q ;

    q_imu.x() = q(1);
    q_imu.y() = q(2);
    q_imu.z() = q(3);
    q_imu.w() = q(0);
    q_rotation.x() = 0.0111389;//-0.0186474
    q_rotation.y() = 0.02011;//-0.0185603
    q_rotation.z() = 0.282201;//-0.486371;
    q_rotation.w() = 0.95908;//-0.873356;   
    q_imu = (q_rotation * q_imu * q_rotation.conjugate()) * q_rotation ;
    q_imu = q_imu.normalized();
    Eigen::Vector3d euler_ang = Quaternion2Euler(q_imu);

    angular_vel.header.stamp=ros::Time::now();
    angular_vel.vector.x = vel(0);
    angular_vel.vector.y = vel(1);
    angular_vel.vector.z = vel(2);

    px4eulerangle.header.stamp=ros::Time::now();
    px4eulerangle.vector.x = euler_ang.x();
    px4eulerangle.vector.y = euler_ang.y();
    px4eulerangle.vector.z = euler_ang.z();
    
    rpyGoal.header.frame_id = "map";
    rpyGoal.pose.orientation.x = q_imu.x();
    rpyGoal.pose.orientation.y = q_imu.y();
    rpyGoal.pose.orientation.z = q_imu.z();
    rpyGoal.pose.orientation.w = q_imu.w();
}

void Correct(const sensor_msgs::MagneticField::ConstPtr& data)
{
    //Accelerometer correction
    Eigen::Vector3d mag(data->magnetic_field.x, data->magnetic_field.y, data->magnetic_field.z);
    /*H_acc << -2 * q(2),  2 * q(3), -2 * q(0),  2 * q(1),
              2 * q(1),  2 * q(0),  2 * q(3),  2 * q(2),
              2 * q(0), -2 * q(1), -2 * q(2),  2 * q(3); 
    h_acc << 2 * q(1) * q(3) - 2 * q(0) * q(2),
             2 * q(0) * q(1) + 2 * q(2) * q(3),
             q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);*/
    H_acc << -2 * q(2),  2 * q(3), -2 * q(0),  2 * q(1),
              2 * q(1),  2 * q(0),  2 * q(3),  2 * q(2),
              0, -2 * q(1), -2 * q(2),  0;          
    h_acc << 2 * q(1) * q(3) - 2 * q(0) * q(2),
             2 * q(0) * q(1) + 2 * q(2) * q(3),
             1 - 2*q(1) * q(1) - 2*q(2) * q(2) ;
    Q_acc << 0.01, 0.0, 0.0, 
             0.0, 0.01, 0.0, 
             0.0, 0.0, 0.01;
    K_acc << P * (H_acc.transpose()) * ((H_acc * P * H_acc.transpose() + Q_acc).inverse());
    q_acc = K_acc * (Accel -  9.80665 * h_acc );    
    q_acc(3) = 0;  
    q = q + q_acc;
    P = (I - K_acc * H_acc) * P;

    //Magnetometer correction
    H_mag <<  2 * q(3),  2 * q(2),  2 * q(1),  2 * q(0),
              2 * q(0), -2 * q(1), -2 * q(2), -2 * q(3),
             -2 * q(1), -2 * q(0),  2 * q(3),  2 * q(2); 
    h_mag << 2 * q(1) * q(2) + 2 * q(0) * q(3),
             q(0) * q(0) - q(1) * q(1) - q(2) * q(2) - q(3) * q(3),
             2 * q(2) * q(3) - 2 * q(0) * q(1);
    Q_mag <<  0.0001, 0.0, 0.0,
              0.0, 0.0001, 0.0,
              0.0, 0.0, 0.0001;
    K_mag << P * H_mag.transpose() * ((H_mag * P * H_mag.transpose() + Q_mag).inverse());

    mag_max[0] = 0.0000393;
    mag_max[1] = 0.0000302;
    mag_max[2] = -0.0000520;
    mag_min[0] = 0.0000312;
    mag_min[1] = 0.0000236;
    mag_min[2] = -0.0000573;
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;

    for(int i = 0; i<2; i++){
        if(mag(i) > mag_max[i])
            mag(i) = mag_max[i];
        if(mag(i) < mag_min[i])
            mag(i) = mag_min[i];
    }

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;
    mag(0) = (mag(0) - mag_bias[0])*65536 * (avg_rad/mag_scale[0]);
    mag(1) = (mag(1) - mag_bias[1])*65536 * (avg_rad/mag_scale[1]); 
    mag(2) = (mag(2) - mag_bias[2])*65536 * (avg_rad/mag_scale[2]);

    mag = mag / sqrt(mag(0)*mag(0)+mag(1)*mag(1)+mag(2)*mag(2));
    q_mag = K_mag * (mag - h_mag);
    q_mag(1) = 0.0;
    q_mag(2) = 0.0;
    q = q + q_mag ;
    q = q / sqrt(q(0)*q(0)+q(1)*q(1)+q(2)*q(2)+q(3)*q(3));
    P = (I - K_mag * H_mag) * P;
/*
    q_imu.x() = q(1);
    q_imu.y() = q(2);
    q_imu.z() = q(3);
    q_imu.w() = q(0);
    q_rotation.x() = -0.0122936;//-0.0186474
    q_rotation.y() = -0.00815857;//-0.0185603
    q_rotation.z() = 0.387877;//-0.486371;
    q_rotation.w() = 0.921593;//-0.873356;   
    q_imu = (q_rotation * q_imu * q_rotation.conjugate()) * q_rotation ;
    q_imu = q_imu.normalized();
    Eigen::Vector3d euler_ang = Quaternion2Euler(q_imu);
    
    px4eulerangle.header.stamp=ros::Time::now();
    px4eulerangle.vector.x = euler_ang.x();
    px4eulerangle.vector.y = euler_ang.y();
    px4eulerangle.vector.z = euler_ang.z();
    
    rpyGoal.header.frame_id = "map";
    rpyGoal.pose.orientation.x = q_imu.x();
    rpyGoal.pose.orientation.y = q_imu.y();
    rpyGoal.pose.orientation.z = q_imu.z();
    rpyGoal.pose.orientation.w = q_imu.w();*/
    
    rpyGoal.header.frame_id = "map";
    rpyGoal.pose.orientation.x = q(1);
    rpyGoal.pose.orientation.y = q(2);
    rpyGoal.pose.orientation.z = q(3);
    rpyGoal.pose.orientation.w = q(0);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle nh;
    ros::Subscriber imu_pred_sub;
    ros::Subscriber imu_corr_sub;
    I.setIdentity();
    P.setIdentity();
    q << 0,
         0,
         0,
         0;
    P << 1.0, 0.0, 0.0, 0.0, 
         0.0, 1.0, 0.0, 0.0, 
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    Q << 0.000001, 0.0, 0.0, 0.0,
         0.0, 0.000001, 0.0, 0.0, 
         0.0, 0.0, 0.000001, 0.0,
         0.0, 0.0, 0.0, 0.000001;
    while (ros::ok())
    {
        pub = nh.advertise<geometry_msgs::PoseStamped>("imu_px4", 0);
        pub_angle = nh.advertise<geometry_msgs::Vector3Stamped>("px4euler", 0);
        pub_vel = nh.advertise<geometry_msgs::Vector3Stamped>("angular_vel", 0);
        imu_pred_sub = nh.subscribe("/mavros/imu/data", 100,Predict);
        //imu_corr_sub = nh.subscribe("/mavros/imu/mag", 1,Correct);
        ros::Rate r(400);
        while(1){
            pub.publish(rpyGoal);
            pub_angle.publish(px4eulerangle);
            pub_vel.publish(angular_vel);
            ros::spinOnce();
            r.sleep();
        }
    }
    return 0;
}   
