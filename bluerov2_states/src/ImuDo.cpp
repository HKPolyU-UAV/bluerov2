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
 * \file imu_do.cpp
 * \date 29/02/2024
 * \author pattylo
 * \copyright (c) Airo-Lab, RCUAS of Hong Kong Polytechnic University
 * \brief source file for imu_based OD
 */

#include "../include/ImuDo.h"

void bluerov2_states::ImuDoNodelet::ground_truth_callback(
    const nav_msgs::Odometry::ConstPtr& msg
)
{
    std::cout<<"hi"<<std::endl;
    return;
}

void bluerov2_states::ImuDoNodelet::imu_callback(
    const sensor_msgs::Imu::ConstPtr& msg
)
{
    imu_buf_manage.lock();
    imu_buf.push(msg);
    imu_buf_manage.unlock();

}

void bluerov2_states::ImuDoNodelet::gps_callback(
    const sensor_msgs::NavSatFix::ConstPtr& msg
)
{
    gps_buf_manage.lock();
    gps_buf.push(msg);
    gps_buf_manage.unlock();
}

void bluerov2_states::ImuDoNodelet::main_spin_callback(const ros::TimerEvent& e)
{
    
}