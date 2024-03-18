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

#include "airo_message/ReferencePreview.h"

#include <ros_utilities/ros_utilities.h>

#include "airo_message/Disturbance.h"

// map definition for convinience
#define COLOR_SUB_TOPIC CAMERA_SUB_TOPIC_A

namespace BLUEROV2_STATES 
{
    typedef struct synced_data{
        bool empty;
        std::pair<std::vector<sensor_msgs::Imu::ConstPtr>, sensor_msgs::NavSatFix::ConstPtr> meas_data;
    }  synced_data;

    class ImuDoNodelet : public nodelet::Nodelet, private RosUtilities
    {
    private:
        nav_msgs::Odometry pose_gt;   

        ros::Subscriber gt_sub, imu_sub, gps_sub;
        ros::Publisher update_pub, imu_pub, esti_dist_pub;
        ros::Timer main_spin_timer;

        std::queue<sensor_msgs::Imu::ConstPtr> imu_buf, imu_calib_buf;
        std::queue<sensor_msgs::NavSatFix::ConstPtr> gps_buf;

        std::mutex imu_buf_manage;
        std::mutex gps_buf_manage;

        synced_data SyncMeas();

        std_msgs::Bool pub_data;

        airo_message::Disturbance esti_dist;
        
        void ImuPredict();
        void EskfProcess();
        void predict();
        void update();

        void DistRawMeas();

        bool tracking_start = false;
        double starting_time = 0;
        double init_time = 0;
        bool got_new_synced = false; 

        void ground_truth_callback(const nav_msgs::Odometry::ConstPtr& msg);
        void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
        void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void main_spin_callback(const ros::TimerEvent& e);
        bool initialization();

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

        void dynamics_parameter_config();
        Eigen::MatrixXd dynamics_C(const Sophus::Vector6d& twist_B);
        Eigen::MatrixXd dynamics_D(const Sophus::Vector6d& twist_B);
        Eigen::MatrixXd dynamics_g(const Eigen::Vector3d& euler);

    };
    PLUGINLIB_EXPORT_CLASS(BLUEROV2_STATES::ImuDoNodelet, nodelet::Nodelet)
}

#endif