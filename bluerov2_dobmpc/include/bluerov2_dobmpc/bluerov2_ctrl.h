#ifndef BLUEROV2_CTRL_H
#define BLUEROV2_CTRL_H

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

#include <geometry_msgs/Wrench.h>

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

#include "ros_utilities/ros_utilities.h"

#include "airo_message/BlueRefPreview.h"
#include "airo_message/Disturbance.h"

using namespace Eigen;

#define s_x 0
#define s_y 1
#define s_z 2
#define s_phi 3
#define s_theta 4
#define s_psi 5
#define s_u 6
#define s_v 7
#define s_w 8
#define s_p 9
#define s_q 10
#define s_r 11

#define MPC 0
#define DOMPC 1
#define PID 2

class BLUEROV2_CTRL : private RosUtilities
{
    private:

// GENERAL STUFF
        // solver-related datatype
        struct SolverInput{
            double x0[BLUEROV2_NX];
            double yref[BLUEROV2_N+1][BLUEROV2_NY];
        };

        struct SolverOutput{
            double u0[BLUEROV2_NU];
            double x1[BLUEROV2_NX];
            double status, kkt_res, cpu_time;
        };

        // ROS message variables
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust0;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust1;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust2;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust3;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust4;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust5;

        nav_msgs::Odometry ref_pose;
        nav_msgs::Odometry error_pose;
        nav_msgs::Odometry esti_pose;

        uuv_gazebo_ros_plugins_msgs::FloatStamped control_input0;
        uuv_gazebo_ros_plugins_msgs::FloatStamped control_input1;
        uuv_gazebo_ros_plugins_msgs::FloatStamped control_input2;
        uuv_gazebo_ros_plugins_msgs::FloatStamped control_input3;

        // weird yaw shit
        float yaw_sum = 0;      // yaw degree as continous number
        float pre_yaw = 0;      // former state yaw degree
        float yaw_diff;         // yaw degree difference in every step
        float yaw_ref;          // yaw degree reference in form of (-pi, pi)
        float yaw_error;        // yaw degree error


        // ros subscriber & publisher
        ros::Subscriber pose_sub;
        ros::Subscriber disturb_esti_sub;
        ros::Subscriber ref_sub;

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

        ros::Timer mainspin_timer;

        bool is_start;
        bool got_path;

        // general control
        int ctrller_type;
        void set_current_yaw_for_ctrl();
        void ctrl_allocate(const SolverOutput& u_out);

        airo_message::BlueRefPreview ref_traj;
        airo_message::BlueRef ref_single_pt;
        airo_message::BlueRef last_ref;
        
        // config
        void ctrl_config(ros::NodeHandle& nh);
        void communi_config(ros::NodeHandle& nh);

        // callbacks
        void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);  // get current position
        void ref_cb(const airo_message::BlueRefPreview::ConstPtr& msg);
        void dist_cb(const airo_message::Disturbance::ConstPtr &msg);
        void mainspin_cb(const ros::TimerEvent& e);

        // vehicle states
        Sophus::SE3d vehicle_SE3_world;
        Sophus::Vector6d vehicle_twist_world;
        Sophus::Vector6d vehicle_twist_body;
        Eigen::Vector3d vehicle_Euler;

        void misc_pub();
        double dt = 0.05;


/*
    Note that the below should be written into sub-classes.
*/
// MPC STUFF
        void mpc_config(
            bool hv_disrub,
            ros::NodeHandle& nh
        );
        // Acados variables
        SolverInput acados_in;
        SolverOutput acados_out;
        double acados_param[BLUEROV2_N+1][BLUEROV2_NP];  // disturbances
        int acados_status;   
        bluerov2_solver_capsule * mpc_capsule = bluerov2_acados_create_capsule();

        // dynamics parameters
        // These should all be written into yaml file
        
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
        double Dl[6] = {-11.7391, -20, -31.8678, -25, -44.9085, -5};
        double Dnl[6] = {-18.18,-21.66,-36.99,-1.55,-1.55,-1.55};
        
        Matrix<double,1,6> M_values;    
        Matrix<double,6,6> M;           // mass matrix
        Matrix<double,6,6> invM;        // inverse mass matrix
        // Matrix<double,1,6> Dl_values;   
        // Matrix<double,6,6> Dl;          // linear hydrodynamic damping force
        Matrix<double,6,6> K;           // propulsion matrix
        Matrix<double,6,1> KAu;         // vehicle's generated forces and moments
        // Matrix<double,18,1> dx;
        Matrix<double,3,1> v_linear_body;   // linear velocity in body frame
        Matrix<double,3,1> v_angular_body;  // angular velocity in body frame
        Matrix<double,3,3> R_ib;            // rotation matrix for linear from inertial to body frame
        Matrix<double,3,3> T_ib;            // rotation matrix for angular from inertial to body frame

        // Acados parameter
        bool AUTO_YAW;
        int READ_WRENCH;        // 0: periodic disturbance; 1: random disturbance; 2: read wrench from text
        bool COMPENSATE_D;       // 0: no compensate; 1: compensate

        // mpc
        void mpc_solve();                                           // solve MPC
        void set_mpc_initial_state();
        void set_ref();
        void convert_refmsg_2_acados(
            const int i, 
            const airo_message::BlueRef ref_current
        );
        void set_last_ref();
        void set_mpc_constraints();
        
// PID STUFF
        void pid_config(ros::NodeHandle& nh);
        // pid
        void pid_solve();
        void pid_4D(
            const Sophus::Vector4d& ref,
            const Sophus::Vector4d& current
        );

        Eigen::Vector3d starting_setpt;

        SolverOutput pid_out;

        // Ks
        Sophus::Vector4d Kp, Ki, Kd;
    
        Sophus::Vector4d Error;
        Sophus::Vector4d Derivative;
        Sophus::Vector4d Integral;
        Sophus::Vector4d prevError;

        Sophus::Vector4d get_pid_ref();
        Sophus::Vector4d get_pid_pose();

        ros::Publisher control_input_pub;
        geometry_msgs::Wrench control_input_to_thrust;
        


        

// DISTURBANCE STUFF
        airo_message::Disturbance esti_disturb;

        

    public:
        BLUEROV2_CTRL(ros::NodeHandle& nh);                         // constructor

};

#endif
