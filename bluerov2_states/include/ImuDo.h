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

#include "essential.h"

#include <sophus/se3.hpp>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pthread.h>
#include <tf/tf.h>
#include <boost/thread.hpp>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <queue>
#include <mutex>

// map definition for convinience
#define COLOR_SUB_TOPIC CAMERA_SUB_TOPIC_A

namespace bluerov2_states 
{

    class ImuDoNodelet : public nodelet::Nodelet
    {
    private:
        nav_msgs::Odometry pose_gt;   

        ros::Subscriber gt_sub, imu_sub, gps_sub;
        ros::Timer main_spin_timer;

        std::queue<sensor_msgs::Imu::ConstPtr> imu_buf;
        std::queue<sensor_msgs::NavSatFix::ConstPtr> gps_buf;

        std::mutex imu_buf_manage;
        std::mutex gps_buf_manage;

        void ground_truth_callback(const nav_msgs::Odometry::ConstPtr& msg);
        void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
        void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void main_spin_callback(const ros::TimerEvent& e);

        virtual void onInit()
        {
            ros::NodeHandle& nh = getMTNodeHandle();
            ROS_INFO("ImuDo Nodelet Initiated...");

            gt_sub = nh.subscribe<nav_msgs::Odometry>
                        ("/bluerov2/pose_gt", 1, &ImuDoNodelet::ground_truth_callback, this);
            
            imu_sub = nh.subscribe<sensor_msgs::Imu>
                        ("/bluerov2/imu", 1, &ImuDoNodelet::imu_callback, this);
            
            gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
                        ("/bluerov2/gps", 1, &ImuDoNodelet::gps_callback, this);

            main_spin_timer = nh.createTimer(
                ros::Duration(0.02), 
                &ImuDoNodelet::main_spin_callback, 
                this
            );
        };

    };
    PLUGINLIB_EXPORT_CLASS(bluerov2_states::ImuDoNodelet, nodelet::Nodelet)
}

#endif