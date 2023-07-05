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
#include <gazebo_msgs/GetLinkState.h>

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

using namespace Eigen;

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

    struct SolverParam{
        double disturbance_x;
        double disturbance_y;
        double disturbance_z;
        double disturbance_phi;
        double disturbance_theta;
        double disturbance_psi;
    };

    struct thrust{
        double t0;
        double t1;
        double t2;
        double t3;
        double t4;
        double t5;
    };

    // ROS message variables
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust0;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust2;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust3;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust4;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust5;
    Euler local_euler;
    pos local_pos;
    thrust current_t;
    nav_msgs::Odometry ref_pose;
    nav_msgs::Odometry error_pose;
    nav_msgs::Odometry esti_pose;
    nav_msgs::Odometry esti_disturbance;
    std::vector<ros::Subscriber> subscribers;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input0;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input2;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input3;
    

    // Acados variables
    SolverInput acados_in;
    SolverOutput acados_out;
    double acados_param[BLUEROV2_NP];  // disturbances
    int acados_status;   
    bluerov2_solver_capsule * mpc_capsule = bluerov2_acados_create_capsule();

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

    // EKF parameters
    Matrix<double,6,1> meas_u;      // inputs
    int n = 18;                     // state dimension
    int m = 18;                     // measurement dimension
    Matrix<double,18,1> meas_y;     // measurement vector
    Matrix<double,1,18> x0;         // initial states
    MatrixXd P0 = MatrixXd::Identity(m, m);     // initial covariance
    Matrix<double,18,1> esti_x;     // estimate states
    Matrix<double,18,18> esti_P;    // estimate covariance
    Matrix<double,1,18> Q_cov;      // process noise value
    Matrix<double,18,18> noise_Q;   // process noise matrix
    MatrixXd noise_R = MatrixXd::Identity(m, m)*dt; // measurement noise matrix
    Matrix<double,1,6> cp;          
    Matrix<double,6,6> Cp;          // constant matrix to replace xddot
    
    // Acados parameter
    std::string REF_TRAJ;
    bool AUTO_YAW;
    SolverParam solver_param;

    // Other variables
    tf::Quaternion tf_quaternion;
    int cout_counter = 0;

    double logger_time;
    float yaw_sum = 0;      // yaw degree as continous number
    float pre_yaw = 0;      // former state yaw degree
    float yaw_diff;         // yaw degree difference in every step
    float yaw_ref;          // yaw degree reference in form of (-pi, pi)
    float yaw_error;        // yaw degree error
        

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

    ros::Publisher esti_pose_pub;
    ros::Publisher esti_disturbance_pub;

    // Trajectory variables
    std::vector<std::vector<double>> trajectory;
    int line_number = 0;
    int number_of_steps = 0;
    
    public:

    bool is_start;

    BLUEROV2_DOB(ros::NodeHandle&);                         // constructor
    Euler q2rpy(const geometry_msgs::Quaternion&);          // quaternion to euler angle
    geometry_msgs::Quaternion rpy2q(const Euler&);          // euler angle to quaternion
    int readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data);     // read trajectory
    void ref_cb(int line_to_read);                          // fill N steps reference points into acados
    void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);  // get current position
    void solve();                                           // solve MPC

    // disturbance observer functions
    void thrusts_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg, int index); // read current thrusts
    void EKF();                                             // EKF predict and update
    MatrixXd f(MatrixXd x, MatrixXd u);                     // system process model
    MatrixXd h(MatrixXd x);                                 // measurement model
    MatrixXd compute_jacobian_F(MatrixXd x, MatrixXd u);    // compute Jacobian of system process model
    MatrixXd compute_jacobian_H(MatrixXd x);                // compute Jacobian of measurement model

};

#endif