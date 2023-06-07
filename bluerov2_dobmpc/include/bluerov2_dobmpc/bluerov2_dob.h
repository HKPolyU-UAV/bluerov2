#ifndef BLUEROV2_DOB_H
#define BLUEROV2_DOB_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <nav_msgs/Odometry.h>
#include <bluerov2_dobmpc/Reference.h>

#include <iostream>
#include <fstream>
#include <cmath>
#include <tuple>

#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include "bluerov2_model/bluerov2_model.h"
#include "acados_solver_bluerov2.h"

class BLUEROV2_DOB{
    private:

    enum SystemStates{
        x = 0,
        y = 1,
        z = 2,
        phi = 3,
        theta = 4,
        psi = 5,
        u = 6,
        v = 7,
        w = 8,
        p = 9,
        q = 10,
        r = 11,
    };

    enum ControlInputs{
        u1 = 0,
        u2 = 1,
        u3 = 2,
        u4 = 3,
    };

    struct SolverInput{
        double x0[BLUEROV2_NX];
        double yref[BLUEROV2_N+1][BLUEROV2_NY];
    };

    struct SolverOutput{
        double u0[BLUEROV2_NU];
        double x1[BLUEROV2_NX];
        double status, kkt_res, cpu_time;
    };

    struct Euler{
        double phi;
        double theta;
        double psi;
    };

    struct SolverParam{
        double disturbance_x;
        double disturbance_y;
        double disturbance_z;
    };

    // ROS message variables
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust0;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust2;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust3;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust4;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust5;
    Euler local_euler;
    Euler ref_euler;
    std::vector<uuv_gazebo_ros_plugins_msgs::FloatStamped> thrusts;

    nav_msgs::Odometry pose_gt;
    geometry_msgs::Quaternion ref_quat;

    // Acados variables
    SolverInput acados_in;
    SolverOutput acados_out;
    double acados_param[BLUEROV2_NP];  // disturbances
    int acados_status;   
    bluerov2_solver_capsule * mpc_capsule = bluerov2_acados_create_capsule();

    // parameter
    std::string REF_TRAJ;
    bool AUTO_YAW;
    SolverParam solver_param;

    // Other variables
    tf::Quaternion tf_quaternion;
    int cout_counter = 0;

    double logger_time;

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

    // Trajectory variables
    std::vector<std::vector<double>> trajectory;
    int line_number = 0;
    int number_of_steps = 0;
    
    public:

    BLUEROV2_DOB(ros::NodeHandle&);
    Euler q2rpy(const geometry_msgs::Quaternion&);
    geometry_msgs::Quaternion rpy2q(const Euler&);
    int readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data);
    void ref_cb(int line_to_read);
    void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);  
    void solve();
};

#endif