/*
    This file is part of bluerov2 - A complete control package for bluerov2 UUV

    bluerov2 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    bluerov2 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with bluerov2.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file imu_do.h
 * \date 29/02/2024
 * \author pattylo
 * \copyright (c) Airo-Lab, RCUAS of Hong Kong Polytechnic University
 * \brief header file for imu_based OD
 */

#ifndef IMU_DO_H
#define IMU_DO_H

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pthread.h>
#include <boost/thread.hpp>

#include <ros_utilities/ros_utilities.h>

#include "airo_message/Disturbance.h"
#include "airo_message/ReferencePreview.h"
#include "uuv_gazebo_ros_plugins_msgs/FloatStamped.h"

// map definition for convinience
#define COLOR_SUB_TOPIC CAMERA_SUB_TOPIC_A

namespace BLUEROV2_STATES 
{
    typedef struct synced_data{
        bool update = false;
        std::pair<std::vector<sensor_msgs::Imu::ConstPtr>, sensor_msgs::NavSatFix::ConstPtr> meas_data;
    }synced_data;

    struct thrust{
        double t0;
        double t1;
        double t2;
        double t3;
        double t4;
        double t5;
    };

    class ImuDoNodelet : public nodelet::Nodelet, private RosUtilities
    {
    private:
        nav_msgs::Odometry pose_gt;   

        ros::Subscriber gt_sub, imu_sub, gps_sub;

        ros::Publisher update_pub, imu_pub, esti_dist_pub, xi_pub, est_pub, wrench_pub;
        ros::Timer main_spin_timer;

        std::queue<sensor_msgs::Imu::ConstPtr> imu_buf, imu_calib_buf;
        std::queue<sensor_msgs::NavSatFix::ConstPtr> gps_buf;

        std::mutex imu_buf_manage;
        std::mutex gps_buf_manage;

        std_msgs::Bool pub_data;

        airo_message::Disturbance esti_dist;

        // vehicle states
        Sophus::SE3d vehicle_SE3_world_gt;
        Sophus::Vector6d vehicle_twist_world_gt;
        Sophus::Vector6d vehicle_twist_body_gt;
        Eigen::Vector3d vehicle_Euler_gt;

        bool tracking_start = false;
        double starting_time = 0;
        double init_time = 0;
        bool got_new_synced = false; 

        void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
        void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void pose_gt_callback(const nav_msgs::Odometry::ConstPtr& msg);
        void main_spin_callback(const ros::TimerEvent& e);

        synced_data SyncMeas();

        virtual void onInit();

//      Dynamics.cpp
        double mass = 11.26;
        double added_mass[6] = {1.7182,0,5.468,0,1.2481,0.4006};
        double Ix = 0.3;
        double Iy = 0.63;
        double Iz = 0.58;
        double Dl[6] = {-11.7391, -20, -31.8678, -25, -44.9085, -5};
        double Dnl[6] = {-18.18,-21.66,-36.99,-1.55,-1.55,-1.55};
        double ZG = 0.02;
        double bouyancy = 0.661618;
        double g_constant = 9.81;
        Eigen::Matrix<double,6,6> K;   
        Eigen::Matrix<double,1,6> M_values;    
        Eigen::Matrix<double,6,6> M;           // mass matrix
        Eigen::Matrix<double,6,6> M_rb;        // rigid body mass matrix
        Eigen::Matrix<double,6,6> M_a;         // added mass matrix
        Eigen::Matrix<double,6,6> invM; 

        Sophus::Vector6d imu_raw_B;

        void thrusts_cb(
            const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg, 
            int index
        ); // read current thrusts
        // thrust current_th;
        Sophus::Vector6d current_th;
        std::vector<ros::Subscriber> th_subs;

        Eigen::Vector3d DistRawMeas();
        void dynamics_parameter_config(ros::NodeHandle& nh);
        Sophus::Vector6d cal_system_wrench();

        Sophus::Vector6d dynamics_Mrb(const Sophus::Vector6d& imu_B);
        Sophus::Vector6d dynamics_Tau(const Sophus::Vector6d& u_B);
        Sophus::Vector6d dynamics_C(const Sophus::Vector6d& twist_B);
        Sophus::Vector6d dynamics_D(const Sophus::Vector6d& twist_B);
        Sophus::Vector6d dynamics_Ma(const Sophus::Vector6d& imu_B);
        Sophus::Vector6d dynamics_g(const Eigen::Vector3d& euler);

//      ESKF.cpp
        int index_lala = 0;
        double dt;
        double t_prev;
        Sophus::SE3d SE3_est_I;
        Eigen::Vector3d v_est_I = Eigen::Vector3d::Zero();
        Eigen::Vector3d b_g_est_B = Eigen::Vector3d::Zero();
        Eigen::Vector3d b_a_est_B = Eigen::Vector3d::Zero();
        Eigen::Vector3d g_vector_est_I = Eigen::Vector3d::Zero();
        Eigen::Vector3d xi_est_I = Eigen::Vector3d::Zero();
        Eigen::Vector3d gps_start, inertial_start;
        Eigen::Vector3d prev_gps_trans_I, curr_gps_trans_I;
        double gps_t_prev;
        
        // error state p, R, v, b_a, b_g, g, xi
        Eigen::Matrix<double, 21, 1> dx = Eigen::Matrix<double, 21, 1>::Zero();
        Eigen::Matrix<double, 21, 21> P_cov = Eigen::Matrix<double, 21, 21>::Zero();

        Eigen::Matrix<double, 21, 21> F_predict = Eigen::Matrix<double, 21, 21>::Zero();
        void set_F(
            Eigen::Matrix<double, 21, 21>& F_,
            const Sophus::Vector6d& imu_data_B_,
            const double& dt_
        );
        Eigen::Matrix<double, 21, 21> Q_process = Eigen::Matrix<double, 21, 21>::Zero();
        
        Eigen::Matrix<double, 12,  1> y_innov = Eigen::Matrix<double, 12, 1>::Zero();
        Eigen::Matrix<double, 12, 21> H_update = Eigen::Matrix<double, 12, 21>::Zero();
        void set_H(
            Eigen::Matrix<double, 12, 21>& H
        );
        Eigen::Matrix<double, 21, 12> K_gain = Eigen::Matrix<double, 21, 12>::Zero();
        Eigen::Matrix<double, 12, 12> R_meas = Eigen::Matrix<double, 12, 12>::Zero();

        void EskfProcess(
            const synced_data& data_to_be_fused
        );
        void predict(
            const Sophus::Vector6d& imu_B
        );
        // void 
        Sophus::SE3d get_processed_gps_pose(const sensor_msgs::NavSatFix& gps_current);
        Eigen::Vector3d get_processed_gps_velo();
        void update(
            const Sophus::SE3d& pose_gps_I,
            const Eigen::Vector3d& velo_gps_I,
            const Eigen::Vector3d& tau_data_B
        );
        void inject(
            const Eigen::Matrix<double, 21, 1>& dx_
        );

//      Config.cpp
        bool got_imu = false;
        bool got_gps = false;
        bool got_gt  = false;
        bool filter_start = false;

        void init_odom(ros::NodeHandle& nh);
        void init_bias(ros::NodeHandle& nh);
        void init_disturb(ros::NodeHandle& nh);
        void init_noise(ros::NodeHandle& nh);
        void init_gps(ros::NodeHandle& nh);
        void eskf_config(ros::NodeHandle& nh);

        void communi_config(ros::NodeHandle& nh);
    };
    PLUGINLIB_EXPORT_CLASS(BLUEROV2_STATES::ImuDoNodelet, nodelet::Nodelet)
}

#endif