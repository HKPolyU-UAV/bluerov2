#ifndef BLUEROV2_DOBMPC_H
#define BLUEROV2_DOBMPC_H

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

class BLUEROV2_DOBMPC{
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

    // ROS message variables
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust0;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust2;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust3;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust4;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust5;
    Euler local_euler;
    Euler target_euler;
    Euler ref_euler;
    std::vector<uuv_gazebo_ros_plugins_msgs::FloatStamped> thrusts;
    // Acados variables
    SolverInput acados_in;
    SolverOutput acados_out;
    double acados_param[BLUEROV2_NP];  // disturbances
    int acados_status;   
    bluerov2_solver_capsule * mpc_capsule = bluerov2_acados_create_capsule();

    // Other variables
    tf::Quaternion tf_quaternion;
    int cout_counter = 0;
    
    public:
    struct SolverParam{
        double disturbance_x;
        double disturbance_y;
        double disturbance_z;
    };

    BLUEROV2_DOBMPC();
    Euler q2rpy(const geometry_msgs::Quaternion&);
    geometry_msgs::Quaternion rpy2q(const Euler&);  
    std::vector<uuv_gazebo_ros_plugins_msgs::FloatStamped> solve(const nav_msgs::Odometry&, const bluerov2_dobmpc::Reference&, const SolverParam&);
};

#endif