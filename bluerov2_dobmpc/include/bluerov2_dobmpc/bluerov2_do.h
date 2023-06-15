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
    ros::Subscriber control_input0_sub;
    ros::Subscriber control_input1_sub;
    ros::Subscriber control_input2_sub;
    ros::Subscriber control_input3_sub;

    // ROS message variables
    //nav_msgs::Odometry pose_gt;
    //nav_msgs::Odometry ref_pose;
    Euler local_euler;
    Euler ref_euler;
    pos local_pos;
    pos ref_pos;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input0;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input2;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input3;

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
    MatrixXd Cv(6,1);
    MatrixXd C(6,6);
    MatrixXd K(6, 6);
    K << 0.707, 0.707, -0.707, -0.707, 0, 0,
       0.707, -0.707, 0.707, -0.707, 0, 0,
       0, 0, 0, 0, 1, 1,
       0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0,
       0.167, -0.167, -0.175, 0.175, 0, 0;
    MatrixXd t(6,1);
    MatrixXd g(6,1);

    // discrete model variables
    MatrixXd Ad; // The 24*24 discrete-time state transition matrix that describes how the state vector evolves from one time step to the next. 
    MatrixXd Bd; //The 24*4 discrete-time input matrix that describes how the control input affects the state vector at each time step. 
    MatrixXd Gd; //The 24*1 discrete-time disturbance matrix that describes how the disturbance term affects the state vector at each time step. 
    MatrixXd Cd; //The 12*24 measurement matrix that describes how the measured state variables relate to the actual state variables.

    // Kalman filter variables
    MatrixXd Q; // process noise covariance matrix
    MatrixXd R; // measurement noise covariance matrix

    // other variables
    tf::Quaternion tf_quaternion;
    std::string package_path = ros::package::getPath("bluerov2_dobmpc");
    std::string YAML_NAME;
    std::string yaml_path;

    // Define system dynamics and measurement model functions
    VectorXd f(VectorXd x, double u); // system dynamics function
    VectorXd h(VectorXd x); // measurement model function

    public:

    BLUEROV2_DO(ros::NodeHandle&);
    void pose_gt_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void ref_pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void control_input0_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg); 
    void control_input1_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg); 
    void control_input2_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg); 
    void control_input3_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg); 
    void c2d_euler();
    VectorXd KalmanFilter(VectorXd z, MatrixXd Q, MatrixXd R, VectorXd x0, MatrixXd P0, VectorXd u);

    void EKF(VectorXd& x, MatrixXd& P, double u, VectorXd y, MatrixXd Q, MatrixXd R);
    VectorXd f(VectorXd x, double u);
    VectorXd h(VectorXd x);
    MatrixXd compute_jacobian_F(VectorXd x, double u);
    MatrixXd compute_jacobian_H(VectorXd x);
};



#endif