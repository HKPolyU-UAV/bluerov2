#ifndef BLUEROV2_INTERFACE_H
#define BLUEROV2_INTERFACE_H

#include "bluerov2_dobmpc/bluerov2_dobmpc.h"

class BLUEROV2_INTERFACE{
    private:    

    // parameter
    std::string REF_TRAJ;
    bool AUTO_YAW;
    BLUEROV2_DOBMPC::SolverParam solver_param;

    // Time
    ros::Time current_time;

    // ros subscriber & publisher
    ros::Subscriber pose_sub;
    ros::Publisher thrust0_pub;
    ros::Publisher thrust1_pub;
    ros::Publisher thrust2_pub;
    ros::Publisher thrust3_pub;
    ros::Publisher thrust4_pub;
    ros::Publisher thrust5_pub;

    ros::Publisher ref_pose_pub;
    ros::Publisher error_pose_pub;

    ros::Publisher control_input0_pub;
    ros::Publisher control_input1_pub;
    ros::Publisher control_input2_pub;
    ros::Publisher control_input3_pub;

    // Messages
    bluerov2_dobmpc::Reference mpc_ref;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust0;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust2;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust3;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust4;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust5;
    std::vector<uuv_gazebo_ros_plugins_msgs::FloatStamped> thrusts;
    nav_msgs::Odometry pose_gt;
    
    geometry_msgs::Quaternion ref_quat;

    // Controller
    BLUEROV2_DOBMPC controller;

    // Trajectory variables
    std::vector<std::vector<double>> trajectory;
    int line_number = 0;
    int number_of_steps = 0;

    // Other variables
    tf::Quaternion tf_quaternion;
    int cout_counter = 0;
    double logger_time;
    bool is_sub;

    public:

    BLUEROV2_INTERFACE(ros::NodeHandle&);
    int readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data);
    geometry_msgs::Quaternion rpy2q(const float& euler_phi, const float& euler_theta, const float& euler_psi);
    void ref_cb(int line_to_read);
    void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void run();

};

#endif