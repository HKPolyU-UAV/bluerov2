#ifndef BLUEROV2_AMPC_H
#define BLUEROV2_AMPC_H

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
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/LinkState.h>

#include <iostream>
#include <fstream>
#include <cmath>
#include <tuple>
#include <iomanip>
#include <random>

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

class BLUEROV2_AMPC{
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

    struct acc{
        double x;
        double y;
        double z;
        double phi;
        double theta;
        double psi;
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

    struct wrench{
        double fx;
        double fy;
        double fz;
        double tx;
        double ty;
        double tz;
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
    // pos pre_pos;
    pos body_pos;
    pos pre_body_pos;
    // acc local_acc;
    acc body_acc;
    thrust current_t;       // /bluerov2/thrusters/<0-5>/thrust
    wrench applied_wrench;
    nav_msgs::Odometry ref_pose;
    nav_msgs::Odometry error_pose;
    nav_msgs::Odometry esti_pose;
    nav_msgs::Odometry esti_disturbance;
    nav_msgs::Odometry applied_disturbance;
    nav_msgs::Odometry esti_added_mass;
    nav_msgs::Odometry esti_damping;
    nav_msgs::Odometry esti_Ndamping;
    nav_msgs::Odometry esti_env;
    std::vector<ros::Subscriber> subscribers;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input0;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input2;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input3;
    gazebo_msgs::ApplyBodyWrench body_wrench;
    
    

    // Acados variables
    SolverInput acados_in;
    SolverOutput acados_out;
    double acados_param[BLUEROV2_N+1][BLUEROV2_NP];  // disturbances
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
    double bouyancy = 0.661618;
    double compensate_coef = 0.032546960744430276;
    double rotor_constant = 0.026546960744430276;
    double added_mass[6] = {1.7182,0,5.468,0,1.2481,0.4006};
    Matrix<double,1,6> M_values;    
    Matrix<double,6,6> M;           // mass matrix
    Matrix<double,6,6> invM;        // inverse mass matrix
    Matrix<double,1,6> Dl_values;   
    Matrix<double,6,6> Dl;          // linear hydrodynamic damping force
    Matrix<double,6,6> K;           // propulsion matrix
    Matrix<double,6,1> KAu;         // vehicle's generated forces and moments
    // Matrix<double,18,1> dx;
    Matrix<double,3,1> v_linear_body;   // linear velocity in body frame
    Matrix<double,3,1> v_angular_body;  // angular velocity in body frame
    Matrix<double,3,3> R_ib;            // rotation matrix for linear from inertial to body frame
    Matrix<double,3,3> T_ib;            // rotation matrix for angular from inertial to body frame

    // EKF parameters
    Matrix<double,6,1> wf_disturbance; // world frame disturbance 
    Matrix<double,6,1> meas_u;      // inputs
    int n = 18;                     // state dimension
    int m = 18;                     // measurement dimension
    Matrix<double,18,1> meas_y;     // measurement vector
    MatrixXd P0 = MatrixXd::Identity(m, m);     // initial covariance
    Matrix<double,18,1> esti_x;     // estimate states
    Matrix<double,18,18> esti_P;    // estimate covariance
    Matrix<double,1,18> Q_cov;      // process noise value
    Matrix<double,18,18> noise_Q;   // process noise matrix
    MatrixXd noise_R = MatrixXd::Identity(m, m)*(pow(dt,4)/4); // measurement noise matrix
    
    // Acados parameter
    std::string REF_TRAJ;
    std::string WRENCH_FX;
    std::string WRENCH_FY;
    std::string WRENCH_FZ;
    std::string WRENCH_TZ;
    bool AUTO_YAW;
    int READ_WRENCH;        // 0: periodic disturbance; 1: random disturbance; 2: read wrench from text
    bool COMPENSATE_D;       // 0: no compensate; 1: compensate
    SolverParam solver_param;

    // RLS-FF parameters
    MatrixXd RLSX_P;        // covariance matrix in RLS
    MatrixXd RLSY_P;
    MatrixXd RLSZ_P;
    MatrixXd RLSK_P;
    MatrixXd RLSM_P;
    MatrixXd RLSN_P;
    double lambda;          // forgetting factor
    double lambda_X;
    double lambda_Y;
    double lambda_Z;
    double lambda_N;
    int numParams = 4;          // number of parameters
    VectorXd theta_X;       // unknown parameter vector
    VectorXd theta_Y;
    VectorXd theta_Z;
    VectorXd theta_K;
    VectorXd theta_M;
    VectorXd theta_N;

    int FF_n = 5;
    int FF_d = 50;

    std::vector<double> Xerror_n;   //prediction error buffer
    std::vector<double> Xerror_d;
    std::vector<double> Yerror_n;
    std::vector<double> Yerror_d;
    std::vector<double> Zerror_n;
    std::vector<double> Zerror_d;
    std::vector<double> Nerror_n;
    std::vector<double> Nerror_d;
    
    double RLSX_F;
    double RLSY_F;
    double RLSZ_F;
    double RLSN_F;

    Matrix<double,6,1> wf_env; // world frame env

    // Other variables
    tf::Quaternion tf_quaternion;
    int cout_counter = 0;
    int rand_counter = 0;
    int fx_counter = 0;
    double dis_time = 0;
    double periodic_counter = 0;
    double amplitudeScalingFactor_X;
    double amplitudeScalingFactor_Y;
    double amplitudeScalingFactor_Z;
    double amplitudeScalingFactor_N;

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
    ros::Publisher applied_disturbance_pub;
    ros::Publisher esti_added_mass_pub;
    ros::Publisher esti_damping_pub;
    ros::Publisher esti_Ndamping_pub;
    ros::Publisher esti_env_pub;
    // ros::Subscriber imu_sub;
    ros::ServiceClient client;

    // Trajectory variables
    std::vector<std::vector<double>> trajectory;
    int line_number = 0;
    int number_of_steps = 0;
    
    public:

    bool is_start;

    BLUEROV2_AMPC(ros::NodeHandle&);                         // constructor
    Euler q2rpy(const geometry_msgs::Quaternion&);          // quaternion to euler angle
    geometry_msgs::Quaternion rpy2q(const Euler&);          // euler angle to quaternion
    int readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data);     // read trajectory
    void ref_cb(int line_to_read);                          // fill N steps reference points into acados
    void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);  // get current position
    void solve();                                           // solve MPC

    // disturbance observer functions
    void thrusts_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg, int index); // read current thrusts
    // void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
    void applyBodyWrench();
    void EKF();
    void RLSFF();
    MatrixXd RLSFF2(VectorXd RLS_x, double RLS_y);
    // void idValidation();  
    MatrixXd RK4(MatrixXd x, MatrixXd u);                                           // EKF predict and update
    MatrixXd f(MatrixXd x, MatrixXd u);                     // system process model
    MatrixXd h(MatrixXd x);                                 // measurement model
    MatrixXd compute_jacobian_F(MatrixXd x, MatrixXd u);    // compute Jacobian of system process model
    MatrixXd compute_jacobian_H(MatrixXd x);                // compute Jacobian of measurement model

};

#endif
