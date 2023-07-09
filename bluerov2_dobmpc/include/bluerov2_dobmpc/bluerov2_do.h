#ifndef BLUEROV2_DO_H
#define BLUEROV2_DO_H

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
// #include <gazebo_msgs/srv/ApplyBodyWrench.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
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
    // struct acc{
    //     double x;
    //     double y;
    //     double z;
    // };
        
    // ROS subsriber & publisher & service
    ros::Subscriber pose_gt_sub;
    // ros::Subscriber imu_sub;
    ros::Publisher esti_pose_pub;
    ros::Publisher esti_disturbance_pub;
    ros::ServiceClient client;

    // ROS message variables
    Euler local_euler;
    pos local_pos;
    // acc local_acc;
    nav_msgs::Odometry esti_pose;
    nav_msgs::Odometry esti_disturbance;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust0;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust2;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust3;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust4;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust5;
    std::vector<ros::Subscriber> subscribers;

    // dynamics parameters
    double dt = 0.05;
    double mass = 11.26;
    double Ix = 0.3;
    double Iy = 0.63;
    double Iz = 0.58;
    double ZG = 0.02;
    double g = 9.81;
    double bouyancy = -0.66;
    double added_mass[6] = {1.7182,0,5.468,0,1.2481,0.4006};
    Matrix<double,1,6> M_values;
    Matrix<double,6,6> M;
    Matrix<double,6,6> invM;
    Matrix<double,6,1> Cv;
    Matrix<double,6,6> C;
    Matrix<double,6,6> K;
    Matrix<double,6,1> KAu;
    Matrix<double,18,1> dx;

    // other variables
    tf::Quaternion tf_quaternion;
    std::string package_path = ros::package::getPath("bluerov2_dobmpc");
    std::string YAML_NAME;
    std::string yaml_path;
    int cout_counter = 0;

    // Define EKF parameters
    Matrix<double,6,1> wf_disturbance; // world frame disturbance 
    Matrix<double,6,1> meas_u;  // input
    int n = 18;                 // state dimension
    int m = 18;                 // measurement dimension
    Matrix<double,18,1> meas_y; //measurement vector

    // Define initial state and covariance
    Matrix<double,1,18> x0;
    MatrixXd P0 = MatrixXd::Identity(m, m);
    Matrix<double,18,1> esti_x;
    Matrix<double,18,18> esti_P;

    // Define process noise and measurement noise covariances
    Matrix<double,1,18> Q_cov;
    Matrix<double,18,18> noise_Q;
    MatrixXd noise_R = MatrixXd::Identity(m, m)*dt;

    // constant matrix
    Matrix<double,1,6> cp;
    Matrix<double,6,6> Cp;

    
    public:

    bool is_start;

    BLUEROV2_DO(ros::NodeHandle&);
    void pose_gt_cb(const nav_msgs::Odometry::ConstPtr& msg);
    // void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
    void thrusts_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg, int index);

    void EKF();
    MatrixXd RK4(MatrixXd x, MatrixXd u);
    MatrixXd f(MatrixXd x, MatrixXd u);
    MatrixXd h(MatrixXd x);
    MatrixXd compute_jacobian_F(MatrixXd x, MatrixXd u);
    MatrixXd compute_jacobian_H(MatrixXd x);

    // void applyBodyWrench();
};



#endif