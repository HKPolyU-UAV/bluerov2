#include "bluerov2_states/ImuDo.h"


void BLUEROV2_STATES::ImuDoNodelet::EskfProcess(
    const synced_data& data_to_be_fused
)
{
    if(imu_buf.empty())
        return;

    if(
        !data_to_be_fused.update
    )
    // if no need to update, propagate nominal state
    {
        predict(
            imumsg_to_accl(
                *imu_buf.back()
            )
        );
        return;
    }    

    sensor_msgs::NavSatFix::ConstPtr gps_msg_temp = data_to_be_fused.meas_data.second;

    predict(
        imumsg_to_accl(
            *data_to_be_fused.meas_data.first.back()
        )
    );

    // std::cout<<"hihihihi"<<std::endl;
    // std::cout<<"before"<<std::endl;
    // std::cout<<SE3_est_I.translation()<<std::endl<<std::endl;
    // std::cout<<v_est_I<<std::endl<<std::endl<<std::endl;

    update(
        get_processed_gps_pose(*gps_msg_temp),
        get_processed_gps_velo(),
        dynamics_Tau(current_th).head(3)
    );

    // std::cout<<"after"<<std::endl;

    // std::cout<<SE3_est_I.translation()<<std::endl<<std::endl;
    // std::cout<<v_est_I<<std::endl<<std::endl;
    // std::cout<<vehicle_twist_world_gt.head(3)<<std::endl<<std::endl;;

    std::cout<<"ERROR HERE"<<std::endl;
    // std::cout<<"======trans========"<<std::endl;
    // std::cout<< 
    //     (
    //         SE3_est_I.inverse()
    //         * 
    //         vehicle_SE3_world_gt
    //     ).log().norm()
    // <<std::endl;
    // std::cout<<"=====velo========"<<std::endl;
    // std::cout<< 
    //     (
    //         v_est_I
    //         - 
    //         vehicle_twist_world_gt.head(3)
    //     ).norm()
    // <<std::endl;
    std::cout<<"======Xi======="<<std::endl;
    std::cout<<SE3_est_I.rotationMatrix() * xi_est_I<<std::endl;
    std::cout<<"==============="<<std::endl;
    Eigen::Vector3d raw_xi = DistRawMeas();
    std::cout<<"==============="<<std::endl<<std::endl<<std::endl;;\

    geometry_msgs::Point xi_data;
    xi_data.x = (SE3_est_I.rotationMatrix() * xi_est_I).x();
    xi_data.y = (SE3_est_I.rotationMatrix() * xi_est_I).y();
    xi_data.z = (SE3_est_I.rotationMatrix() * xi_est_I).z();
    
    xi_pub.publish(xi_data);

    // esti_dist.disturb.linear.x = xi_est_I.x();
    // esti_dist.disturb.linear.y = xi_est_I.y();
    // esti_dist.disturb.linear.z = xi_est_I.z();

    raw_xi = vehicle_SE3_world_gt.rotationMatrix().inverse() * Eigen::Vector3d(40, 40, 10);

    esti_dist.disturb.linear.x = raw_xi.x();
    esti_dist.disturb.linear.y = raw_xi.y();
    esti_dist.disturb.linear.z = raw_xi.z();

    // if((xi_est_I - raw_xi).norm() < 10.0)
        esti_dist_pub.publish(esti_dist);
    
    index_lala ++;
    // if(index_lala >= 100)
    //     patty::Debug("OUTTA HJERE");
}

void BLUEROV2_STATES::ImuDoNodelet::predict(
    const Sophus::Vector6d& imu_data_B_
)
{
    dt = ros::Time::now().toSec() - t_prev;

    dt = 1.0/50.0;

    // std::cout<<"======="<<std::endl;
    // std::cout<<SE3_est_I.translation()<<std::endl;
    
    // state propagation
    SE3_est_I.translation() = SE3_est_I.translation()
        + v_est_I * dt
        + 0.5 * (
            SE3_est_I.rotationMatrix()
            *
            (imu_data_B_.head(3) - b_a_est_B)
        ) * dt * dt
        + 0.5 * g_vector_est_I * dt * dt;

    // std::cout<<"delta velo"<<std::endl;
    // std::cout<<SE3_est_I.rotationMatrix() * (imu_data_B_.head(3) - b_a_est_B) * dt
    //     + g_vector_est_I * dt<<std::endl<<std::endl;
    // std::cout<<
    //     SE3_est_I.rotationMatrix() * (imu_data_B_.head(3) - b_a_est_B) + g_vector_est_I
    // <<std::endl<<std::endl;;

    v_est_I = v_est_I 
        + SE3_est_I.rotationMatrix() * (imu_data_B_.head(3) - b_a_est_B) * dt
        + g_vector_est_I * dt;
    
    SE3_est_I.so3() = 
        SE3_est_I.so3() 
        * Sophus::SO3d::exp(
            (imu_data_B_.tail(3) - b_g_est_B) * dt
        );

    set_F(F_predict, imu_data_B_, dt);

    dx = F_predict * dx;
    P_cov = F_predict * P_cov.eval() * F_predict.transpose() + Q_process;
    // std::cout<<Q_process<<std::endl<<std::endl;;

    t_prev = ros::Time::now().toSec();
}

void BLUEROV2_STATES::ImuDoNodelet::set_F(
    Eigen::Matrix<double, 21, 21>& F_,
    const Sophus::Vector6d& imu_data_B_,
    const double& dt_
)
{
    F_.setIdentity();

    F_.block<3,3>(0, 3) = Eigen::Matrix3d::Identity() * dt_;
    F_.block<3,3>(3, 6) = - SE3_est_I.so3().matrix() * Sophus::SO3d::hat(imu_data_B_.head(3) - b_a_est_B) * dt_;
    F_.block<3,3>(3,12) = - SE3_est_I.rotationMatrix() * dt_;
    F_.block<3,3>(3,15) = Eigen::Matrix3d::Identity() * dt_;
    F_.block<3,3>(6, 6) = Sophus::SO3d::exp(- (imu_data_B_.tail(3) - b_g_est_B) * dt_).matrix();
    F_.block<3,3>(6, 9) = - Eigen::Matrix3d::Identity() * dt_;
}

Sophus::SE3d BLUEROV2_STATES::ImuDoNodelet::get_processed_gps_pose(
    const sensor_msgs::NavSatFix& gps_current
)
{
    double latMid = gps_current.latitude;
    double m_per_deg_lon = (M_PI/ 180) * 6367449 * cos ( latMid * M_PI / 180);
    double m_per_deg_lat = 111132.954 - 559.822 * cos( 2.0 * latMid * M_PI / 180) + 1.175 * cos( 4.0 * latMid * M_PI / 180);

    Eigen::Vector3d current_trans_I = Eigen::Vector3d(
        inertial_start.x() + m_per_deg_lon * (gps_current.longitude - gps_start.x()),
        inertial_start.y() + m_per_deg_lat * (gps_current.latitude  - gps_start.y()),
        inertial_start.z() + gps_current.altitude  - gps_start.z()
    );

    curr_gps_trans_I = current_trans_I;

    return Sophus::SE3d(
        vehicle_SE3_world_gt.rotationMatrix(),
        current_trans_I
    );
}

Eigen::Vector3d BLUEROV2_STATES::ImuDoNodelet::get_processed_gps_velo()
{
    Eigen::Vector3d velo_gps_meas = (curr_gps_trans_I - prev_gps_trans_I) / (1.0/30.0);
    
    gps_t_prev = ros::Time::now().toSec();
    prev_gps_trans_I = curr_gps_trans_I;
    std::cout<<"=========GPS VELO==========="<<std::endl;
    std::cout<<velo_gps_meas<<std::endl<<std::endl;;
    std::cout<<vehicle_twist_world_gt.head(3)<<std::endl<<std::endl;

    return velo_gps_meas;

    // return vehicle_twist_world_gt.head(3);
}

void BLUEROV2_STATES::ImuDoNodelet::update(
    const Sophus::SE3d& gps_pose_data_I,
    const Eigen::Vector3d& gps_velo_data_I,
    const Eigen::Vector3d& tau_data_B
)
{
    y_innov.setZero();

    y_innov.head(3) =  gps_pose_data_I.translation() - SE3_est_I.translation();
    y_innov.segment<3>(3) = gps_velo_data_I - v_est_I;
    y_innov.segment<3>(6) = (SE3_est_I.so3().inverse() * gps_pose_data_I.so3()).log();    

    Eigen::Vector3d v_est_B = SE3_est_I.rotationMatrix().inverse() * v_est_I;
    // Eigen::Vector3d v_est_B = SE3_est_I.rotationMatrix().inverse() * vehicle_twist_world_gt.head(3);

    std::cout<<"=====R========"<<std::endl;
    std::cout<<(SE3_est_I.so3().inverse() * vehicle_SE3_world_gt.so3()).log().norm() <<std::endl<<std::endl;

    std::cout<<"=====velo_B========"<<std::endl;
    std::cout<<v_est_B<<std::endl<<std::endl;
    std::cout<<vehicle_twist_body_gt.head(3)<<std::endl<<std::endl;
    std::cout<< 
        (
            v_est_B
            - 
            vehicle_twist_body_gt.head(3)
        ).norm()
    <<std::endl;

    // imu_raw_B = 

    y_innov.tail(3) = 
        tau_data_B 
        - (
            dynamics_Mrb(
                imu_raw_B - 
                (
                    Sophus::Vector6d() 
                    << 
                    b_a_est_B, 
                    b_g_est_B
                ).finished() 
            ).head(3)
            - xi_est_I
            + dynamics_Ma(
                imu_raw_B - 
                (
                    Sophus::Vector6d() 
                    << 
                    b_a_est_B, 
                    b_g_est_B
                ).finished() 
            ).head(3)
            // + dynamics_C(
            //     (
            //         Sophus::Vector6d() <<
            //         v_est_B,
            //         imu_raw_B.tail(3)
            //     ).finished()
            // ).head(3)
            + dynamics_D(
                (
                    Sophus::Vector6d() <<
                    v_est_B,
                    imu_raw_B.tail(3)
                ).finished()
            ).head(3)
            // + dynamics_D(
            //     vehicle_twist_body_gt
            // ).head(3)
            + dynamics_g(
                q2rpy(
                    SE3_est_I.unit_quaternion()
                )
            ).head(3)
        ); 

    set_H(H_update);

    // std::cout<<P_cov <<std::endl<<std::endl;

    K_gain = P_cov 
        * H_update.transpose() 
        * (
            H_update 
            * P_cov 
            * H_update.transpose() 
            + R_meas
        ).inverse();

    
    dx = K_gain * y_innov;
    P_cov = (Eigen::Matrix<double, 21, 21>::Identity() - K_gain * H_update) * P_cov;

    inject(dx);
    dx.setZero();

    std_msgs::Header head_temp;
    nav_msgs::Odometry lala = SE3_to_odommsg(
        SE3_est_I, 
        (
            Sophus::Vector6d()
            <<
            // SE3_est_I.rotationMatrix().inverse() *
             v_est_I, 
            Eigen::Vector3d::Zero()
        ).finished(),
        head_temp); 
    est_pub.publish(lala);
}

void BLUEROV2_STATES::ImuDoNodelet::set_H(
    Eigen::Matrix<double, 12, 21>& H_
)
{
    H_.setZero();
    H_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    H_.block<3,3>(3,3) = Eigen::Matrix3d::Identity();
    H_.block<3,3>(6,6) = Eigen::Matrix3d::Identity();
    H_.block<3,3>(9,18) = -Eigen::Matrix3d::Identity();
    // std::cout<<H_<<std::endl;
}

void BLUEROV2_STATES::ImuDoNodelet::inject(
    const Eigen::Matrix<double, 21, 1>& dx_
)
{
    SE3_est_I.translation() = SE3_est_I.translation() + dx_.head(3);
    v_est_I = v_est_I + dx_.segment<3>(3) + dx_.segment<3>(3);
    SE3_est_I.so3() = SE3_est_I.so3() * Sophus::SO3d::exp(dx_.segment<3>(6));

    xi_est_I = xi_est_I + dx_.segment<3>(18);

    // std::cout<<"\ninjection"<<std::endl;
    // std::cout<<dx_.segment<3>(18)<<std::endl;
}