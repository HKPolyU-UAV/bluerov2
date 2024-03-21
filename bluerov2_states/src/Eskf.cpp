#include "bluerov2_states/ImuDo.h"


void BLUEROV2_STATES::ImuDoNodelet::EskfProcess()
{
    // predict();
    // update();
    update_pub.publish(pub_data);
}

void BLUEROV2_STATES::ImuDoNodelet::predict(
    const Sophus::Vector6d& imu_data_B_
)
{
    dt = ros::Time::now().toSec() - t_prev;
    
    // state propagation
    SE3_est_I.translation() = SE3_est_I.translation()
        + v_est_I * dt
        + 0.5 * (
            SE3_est_I.rotationMatrix()
            *
            (imu_data_B_.head(3) - b_a_est_B)
        ) * dt * dt
        + 0.5 * g_vector_est_I * dt * dt;

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

void BLUEROV2_STATES::ImuDoNodelet::update(
    const Sophus::SE3d& gps_data_I,
    const Eigen::Vector3d& tau_data_B
)
{
    y_innov.setZero();

    y_innov.head(3) =  gps_data_I.translation() - SE3_est_I.translation();
    y_innov.segment<3>(3) = (SE3_est_I.so3().inverse() * gps_data_I.so3()).log();
    y_innov.tail(3) = 
        tau_data_B 
        - (
            dynamics_Mrb(imu_raw_B).head(3)
            - xi_est_I
            + dynamics_Ma(imu_raw_B).head(3)
            + dynamics_C(
                (
                    Sophus::Vector6d() <<
                    v_est_I,
                    imu_raw_B.tail(3)
                ).finished()
            ).head(3)
            + dynamics_D(
                (
                    Sophus::Vector6d() <<
                    v_est_I,
                    imu_raw_B.tail(3)
                ).finished()
            ).head(3)
            + dynamics_g(
                q2rpy(
                    SE3_est_I.unit_quaternion()
                )
            ).head(3)
        ); 

    set_H(H_update);
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
}

void BLUEROV2_STATES::ImuDoNodelet::set_H(
    Eigen::Matrix<double, 9, 21>& H_
)
{
    H_.setZero();
    H_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    H_.block<3,3>(3,6) = Eigen::Matrix3d::Identity();
    H_.block<3,3>(18,18) = Eigen::Matrix3d::Identity();
}

void BLUEROV2_STATES::ImuDoNodelet::inject(
    const Eigen::Matrix<double, 21, 1>& dx_
)
{
    SE3_est_I.translation() = SE3_est_I.translation() + dx_.head(3);
    v_est_I = v_est_I + dx_.segment<3>(3) + dx_.segment<3>(3);
    SE3_est_I.so3() = SE3_est_I.so3() * Sophus::SO3d::exp(dx_.segment<3>(6));

    xi_est_I = xi_est_I + dx_.segment<3>(18);
}