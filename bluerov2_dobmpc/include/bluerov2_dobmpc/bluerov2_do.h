#ifndef BLUEROV2_DO_H
#define BLUEROV2_DO_H

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
using namespace Eigen;

class BLUEROV2_DO{
    private:
    struct Euler{
            double phi;
            double theta;
            double psi;
        };
    struct pos{
            double x;
            double y;
            double z;
            double u;
            double v;
            double w;
            double p;
            double q;
            double r;
        };
        
    // ROS subsriber
    ros::Subscriber pose_gt_sub;
    ros::Subscriber ref_pose_sub;
    ros::Subscriber thrust0_sub;
    ros::Subscriber thrust1_sub;
    ros::Subscriber thrust2_sub;
    ros::Subscriber thrust3_sub;
    ros::Subscriber thrust4_sub;
    ros::Subscriber thrust5_sub;

    // ROS message variables
    //nav_msgs::Odometry pose_gt;
    //nav_msgs::Odometry ref_pose;
    Euler local_euler;
    Euler ref_euler;
    pos local_pos;
    pos ref_pos;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust0;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust2;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust3;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust4;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust5;

    // system parameters
    double dt = 0.05;
    float m = 11.26;
    float Ix = 0.3;
    float Iy = 0.63;
    float Iz = 0.58;
    float ZG = 0.02;
    float g = 9.81;
    float bouyancy = 0.2*g;
    float added_mass[6] = {1.7182,0,5.468,0,1.2481,0.4006};
    VectorXd M_values(6);
    M_values << m + added_mass[0], m + added_mass[1], m + added_mass[2], Ix + added_mass[3], Iy + added_mass[4], Iz + added_mass[5];
    MatrixXd M = M_values.asDiagonal();
    MatrixXd invM = M.inverse();
    MatrixXd Cv(6,1);
    MatrixXd C(6,6);
    MatrixXd K(6, 6);
    K << 0.707, 0.707, -0.707, -0.707, 0, 0,
       0.707, -0.707, 0.707, -0.707, 0, 0,
       0, 0, 0, 0, 1, 1,
       0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0,
       0.167, -0.167, -0.175, 0.175, 0, 0;
    VectorXd KAu(6);

    // other variables
    tf::Quaternion tf_quaternion;
    std::string package_path = ros::package::getPath("bluerov2_dobmpc");
    std::string YAML_NAME;
    std::string yaml_path;

    // Define system parameters
    VectorXd u(6); // input
    double dt = 0.05; // time step
    int n = 18; // state dimension
    int m = 18; // measurement dimension
    VectorXd meas_y(m); //measurement vector

    // Define initial state and covariance
    VectorXd x0(n);
    X0.fill(0);
    //x0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // initial state
    
    P0 << MatrixXd::Identity(n,n);  // initial covariance
    VectorXd esti_x(n);
    MatrixXd esti_P(n, n);

    // Define process noise and measurement noise covariances
    MatrixXd noise_Q(n, n);
    /*
    noise_Q << 0.25*dt^4, 0.5*dt^3, 0, 0, 0, 0,
                0.5*dt^3, dt^2, 0, 0, 0, 0,
                0, 0, 0.25*dt^4, 0.5*dt^3, 0, 0,
                0, 0, 0,5*dt^3, dt^2, 0, 0,
                0, 0, 0, 0, 0.25*dt^4, 0.5*dt^3,
                0, 0, 0, 0, 0.5*dt^3, dt^2; // process noise covariance
    */
    MatrixXd noise_R(m, m);
    /*
    noise_R << dt, 0, 0, 0, 0, 0,
                0, dt, 0, 0, 0, 0,
                0, 0, dt, 0, 0, 0,
                0, 0, 0, dt, 0, 0,
                0, 0, 0, 0, dt, 0,
                0, 0, 0, 0, 0, dt; // measurement noise covariance
    */
    public:

    BLUEROV2_DO(ros::NodeHandle&);
    void pose_gt_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void ref_pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void thrust0_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg); 
    void thrust1_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg); 
    void thrust2_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg); 
    void thrust3_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg); 
    void thrust4_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg); 
    void thrust5_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg); 
    // void c2d_euler();
    // VectorXd KalmanFilter(VectorXd z, MatrixXd Q, MatrixXd R, VectorXd x0, MatrixXd P0, VectorXd u);

    void EKF();
    VectorXd f(VectorXd x, VectorXd u);
    VectorXd h(VectorXd x);
    MatrixXd compute_jacobian_F(VectorXd x, double u);
    MatrixXd compute_jacobian_H(VectorXd x);
};



#endif