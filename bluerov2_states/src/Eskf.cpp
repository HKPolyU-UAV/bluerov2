#include "bluerov2_states/ImuDo.h"

void BLUEROV2_STATES::ImuDoNodelet::EskfProcess()
{
    // predict();
    update();
    update_pub.publish(pub_data);
}

void BLUEROV2_STATES::ImuDoNodelet::predict(
    const Sophus::Vector6d& imu_B
)
{
    dt = ros::Time::now().toSec() - t_prev;
    
    // state propagation
    SE3_est_I.translation() = SE3_est_I.translation()
        + v_I * dt
        + 0.5 * (
            SE3_est_I.rotationMatrix()
            *
            (imu_B.head(3) - b_a_B)
        ) * dt * dt
        + 0.5 * g_vector_I * dt * dt;

    v_I = v_I 
        + SE3_est_I.rotationMatrix() * (imu_B.head(3) - b_a_B) * dt
        + g_vector_I * dt;
    
    SE3_est_I.so3() = 
        SE3_est_I.so3() 
        * Sophus::SO3d::exp(
            (imu_B.tail(3) - b_g_B) * dt
        );

    F_predict.setIdentity();

    F_predict.block<3,3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    F_predict.block<3,3>(3, 6) = - SE3_est_I.so3().matrix() * Sophus::SO3d::hat(imu_B.head(3) - b_a_B) * dt;
    ;
    F_predict.block<3,3>(3,12) = - SE3_est_I.rotationMatrix() * dt;
    F_predict.block<3,3>(3,15) = Eigen::Matrix3d::Identity() * dt;
    F_predict.block<3,3>(6, 6) = Sophus::SO3d::exp(- (imu_B.tail(3) - b_g_B) * dt).matrix();
    F_predict.block<3,3>(6, 9) = - Eigen::Matrix3d::Identity() * dt;

    dx = F_predict * dx;
    P_cov = F_predict * P_cov.eval() * F_predict.transpose() + Q_process;

    t_prev = ros::Time::now().toSec();
}

void BLUEROV2_STATES::ImuDoNodelet::update()
{
    H_update.setZero();
    K_gain.setZero();

    H_update.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    H_update.block<3,3>(3,6) = Eigen::Matrix3d::Identity();

    R_meas = (
        Eigen::Matrix<double, 9, 1>() 
        << 
        p_meas_noise, 
        p_meas_noise,
        p_meas_noise,
        r_meas_noise,
        r_meas_noise,
        r_meas_noise,
        th_meas_noise,
        th_meas_noise,
        th_meas_noise
    ).finished().asDiagonal();
}