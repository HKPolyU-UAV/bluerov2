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

#include "bluerov2_states/ImuDo.h"

void BLUEROV2_STATES::ImuDoNodelet::imu_callback(
    const sensor_msgs::Imu::ConstPtr& msg
)
{
    if(!got_imu)
        got_imu = true;

    if(!filter_start)
        return;

    imu_raw_B = imumsg_to_accl(*msg);

    imu_buf_manage.lock();
    imu_buf.push(msg);
    imu_buf_manage.unlock();

}

void BLUEROV2_STATES::ImuDoNodelet::gps_callback(
    const sensor_msgs::NavSatFix::ConstPtr& msg
)
{
    if(!got_gps)
        got_gps = true;

    if(!filter_start)
        return;

    gps_buf_manage.lock();
    gps_buf.push(msg);
    gps_buf_manage.unlock();
}

void BLUEROV2_STATES::ImuDoNodelet::pose_gt_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
    if(!got_gt)
        got_gt = true;

    vehicle_SE3_world_gt = posemsg_to_SE3(odom->pose.pose);
    vehicle_twist_world_gt = twistmsg_to_velo(odom->twist.twist);
    
    vehicle_twist_body_gt.head(3) = 
        vehicle_SE3_world_gt.rotationMatrix().inverse() * vehicle_twist_world_gt.head(3);
    vehicle_twist_body_gt.tail(3) = 
        Jacobi3dR(vehicle_SE3_world_gt).inverse() * vehicle_twist_world_gt.tail(3);

    vehicle_Euler_gt = q2rpy(vehicle_SE3_world_gt.unit_quaternion());
}

void BLUEROV2_STATES::ImuDoNodelet::main_spin_callback(const ros::TimerEvent& e)
{   
    ROS_GREEN_STREAM("HERE");
    
    DistRawMeas();

    if(!filter_start)
        return;

    // if(!tracking_start)
    // {
    //     // tracking_start = initialization();
    //     tracking_start = true;

    //     return;
    // }
    
    // synced_data temp = SyncMeas();

    // if(!temp.empty)
    // {
    //     ROS_RED_STREAM("DO IMU PREDICT/PUB");
    //     ImuPredict();

    //     return;
    // }

    // ROS_GREEN_STREAM("DO ESKF UPDATE");
    // EskfProcess();    
}


bool BLUEROV2_STATES::ImuDoNodelet::initialization()
{
    if(
        (ros::Time::now().toSec() - starting_time) 
        > 
        ros::Duration(init_time).toSec()
    )
        return true;
    
    return false;
}


BLUEROV2_STATES::synced_data BLUEROV2_STATES::ImuDoNodelet::SyncMeas()
// reference: VINS-Mono
{
    using namespace std;

    std::cout<<"sync"<<std::endl;
    synced_data meas_return;
    meas_return.empty = true;

    if (imu_buf.empty() || gps_buf.empty())
        // if not, we return null vector
        return meas_return;

    if (
        !(imu_buf.back()->header.stamp.toSec() > gps_buf.front()->header.stamp.toSec())
    ) // we want the last_imu_time always > gps_first_time
    {
        // if not, we return null vector
        return meas_return;
    }

    cout<<1<<endl;

    if (
        !(imu_buf.front()->header.stamp.toSec() < gps_buf.front()->header.stamp.toSec())
    ) // we want the first_imu_time always < gps_first_time
    {
        // if not, throw away the 
        gps_buf.pop();

        // return meas_;
    }

    cout<<2<<endl;
    
    sensor_msgs::NavSatFix::ConstPtr gps_msg_temp = gps_buf.front();
    gps_buf.pop();
    
    std::vector<sensor_msgs::Imu::ConstPtr> Imu_buff_temp;
    while (imu_buf.front()->header.stamp.toSec() < gps_msg_temp->header.stamp.toSec())
    // collect all the imu data before the updated GPS signal!
    {
        Imu_buff_temp.emplace_back(imu_buf.front());
        imu_buf.pop();
    }

    if (Imu_buff_temp.empty())
        ROS_WARN("NO IMU DATA BETWEEN 2 UPDATES!");

    meas_return.empty = false;
    meas_return.meas_data = std::make_pair(Imu_buff_temp,gps_msg_temp);
    
    std::cout<<"sync"<<std::endl;
    return meas_return;
}

void BLUEROV2_STATES::ImuDoNodelet::onInit()
{
    ros::NodeHandle& nh = getMTNodeHandle();
    ROS_INFO("ImuDo Nodelet Initiated...");

    starting_time = ros::Time::now().toSec();

    dynamics_parameter_config(nh);
    // eskf_config(nh);

    communi_config(nh);
    
    filter_start = true;
}