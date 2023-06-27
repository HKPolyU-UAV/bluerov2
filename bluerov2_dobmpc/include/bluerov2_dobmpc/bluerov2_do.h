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
#include <eigen3/Eigen/Dense>
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
    float mass = 11.26;
    float Ix = 0.3;
    float Iy = 0.63;
    float Iz = 0.58;
    float ZG = 0.02;
    float g = 9.81;
    float bouyancy = 0.2*g;
    float added_mass[6] = {1.7182,0,5.468,0,1.2481,0.4006};
    Matrix<float,1,6> M_values;
    Matrix<float,6,6> M;
    Matrix<float,6,6> invM;
    Matrix<float,6,1> Cv;
    Matrix<float,6,6> C;
    Matrix<float,6,6> K;
    Matrix<float,1,6> KAu;

    // other variables
    tf::Quaternion tf_quaternion;
    std::string package_path = ros::package::getPath("bluerov2_dobmpc");
    std::string YAML_NAME;
    std::string yaml_path;

    // Define system parameters
    Matrix<float,1,6> u; // input
    int n = 18; // state dimension
    int m = 18; // measurement dimension
    Matrix<float,1,18> meas_y; //measurement vector

    // Define initial state and covariance
    Matrix<float,1,18> x0;
    MatrixXd P0 = MatrixXd::Identity(m, m);
    Matrix<float,1,18> esti_x;
    Matrix<float,18,18> esti_P;

    // Define process noise and measurement noise covariances
    Matrix<float,1,18> R_cov;
    Matrix<float,18,18> noise_Q;
    /*
    noise_Q << 0.25*dt^4, 0.5*dt^3, 0, 0, 0, 0,
                0.5*dt^3, dt^2, 0, 0, 0, 0,
                0, 0, 0.25*dt^4, 0.5*dt^3, 0, 0,
                0, 0, 0,5*dt^3, dt^2, 0, 0,
                0, 0, 0, 0, 0.25*dt^4, 0.5*dt^3,
                0, 0, 0, 0, 0.5*dt^3, dt^2; // process noise covariance
    */
    MatrixXd noise_R = MatrixXd::Identity(m, m)*dt;
    MatrixXd Cp = MatrixXd::Identity(6, 6);
    
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

    void EKF();
    VectorXd f(VectorXd x, VectorXd u);
    VectorXd h(VectorXd x);
    MatrixXd compute_jacobian_F(VectorXd x, VectorXd u);
    MatrixXd compute_jacobian_H(VectorXd x);
};



#endif