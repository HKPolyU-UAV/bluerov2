#include "bluerov2_states/ImuDo.h"

Eigen::Vector3d BLUEROV2_STATES::ImuDoNodelet::DistRawMeas()
{
    
    Sophus::Vector6d sys_wrench_B = cal_system_wrench();

    // M * imu_B = sys_wrench_B + delta_B
    Sophus::Vector6d delta_raw_B = M_rb * imu_raw_B - sys_wrench_B;

    std::cout<<"===================="<<std::endl;
    // std::cout<<"FORCE RAW"<<std::endl;
    // std::cout<<(M * imu_raw_B).head(3)<<std::endl;
    std::cout<<"DELTA RAW"<<std::endl;
    std::cout<<
        q_rotate_vector(
            vehicle_SE3_world_gt.unit_quaternion(),
            delta_raw_B.head(3)
        )
    <<std::endl;
    std::cout<<"===================="<<std::endl;

    // std::cout<<delta_raw_B.head(3)<<std::endl;

    // std::cout<<imu_raw_B - vehicle_twist_body_gt<<std::endl;

    // esti_dist.disturb.linear.x = delta_raw_B(0);
    // esti_dist.disturb.linear.y = delta_raw_B(1);
    // esti_dist.disturb.linear.z = delta_raw_B(2);

    Eigen::Vector3d delta_I = Eigen::Vector3d(10, 10, 10);
    Eigen::Vector3d delta_B = vehicle_SE3_world_gt.rotationMatrix().inverse() * delta_I;

    // esti_dist.disturb.linear.x = delta_B.x();
    // esti_dist.disturb.linear.y = delta_B.y();
    // esti_dist.disturb.linear.z = delta_B.z();

    // esti_dist_pub.publish(esti_dist);

    geometry_msgs::Point wrench_data;
    Eigen::Vector3d delta_raw_I = q_rotate_vector(
        vehicle_SE3_world_gt.unit_quaternion(),
        delta_raw_B.head(3)
    );

    wrench_data.x = delta_raw_I.x();
    wrench_data.y = delta_raw_I.y();
    wrench_data.z = delta_raw_I.z();

    wrench_pub.publish(wrench_data);

    return delta_raw_B.head(3);
}

Sophus::Vector6d BLUEROV2_STATES::ImuDoNodelet::cal_system_wrench()
{
    // std::cout<<"hihi"<<std::endl;
    // std::cout<<dynamics_g(vehicle_Euler_gt)<<std::endl;

    return 
        dynamics_Tau(current_th)
        // -
        // dynamics_C(vehicle_twist_body_gt)
        // do not know why, but currently, it affects the result
        -
        dynamics_D(vehicle_twist_body_gt)
        -
        dynamics_Ma(imu_raw_B)
        -
        dynamics_g(vehicle_Euler_gt)
        ;
}

Sophus::Vector6d BLUEROV2_STATES::ImuDoNodelet::dynamics_Mrb(
    const Sophus::Vector6d& twist_B
)
{
    return M_rb * twist_B;
}


Sophus::Vector6d BLUEROV2_STATES::ImuDoNodelet::dynamics_Tau(
    const Sophus::Vector6d& u
)
{
    return K * u;
}

Sophus::Vector6d BLUEROV2_STATES::ImuDoNodelet::dynamics_C(
    const Sophus::Vector6d& twist_B
)
{
    // std::cout<<twist_B<<std::endl<<std::endl;
    Eigen::Matrix<double,6,6> C_rb;

    C_rb.setZero();
    // C_rb.block<3,3>(0,3) = Sophus::SO3d::hat(-mass * twist_B.head(3));
    // C_rb.block<3,3>(0,3) = Sophus::SO3d::hat(- mass * twist_B.head(3));
    // C_rb.block<3,3>(3,3) = Sophus::SO3d::hat(
    //     (
    //         Eigen::Vector3d()
    //         <<
    //         - Ix * twist_B(3),
    //         - Ix * twist_B(4),
    //         - Ix * twist_B(5)
    //     ).finished()
    // );

    // std::cout<<C_rb<<std::endl<<std::endl;

    Eigen::Matrix<double,6,6> C_a;
    C_a.setZero();
    C_a.block<3,3>(0,3) = Sophus::SO3d::hat(
        (
            Eigen::Vector3d()
            <<
            added_mass[0] * twist_B(0),
            added_mass[1] * twist_B(1),
            added_mass[2] * twist_B(2)
        ).finished()
    );
    // C_a.block<3,3>(0,3) = Sophus::SO3d::hat(
    //     (
    //         Eigen::Vector3d()
    //         <<
    //         added_mass[0] * twist_B(0),
    //         added_mass[1] * twist_B(1),
    //         added_mass[2] * twist_B(2)
    //     ).finished()
    // );
    // C_a.block<3,3>(3,3) = Sophus::SO3d::hat(
    //     (
    //         Eigen::Vector3d()
    //         <<
    //         added_mass[3] * twist_B(3),
    //         added_mass[3] * twist_B(4),
    //         added_mass[3] * twist_B(5)
    //     ).finished()
    // );

    // std::cout<<C_a<<std::endl<<std::endl;

    // std::cout<<"===================="<<std::endl;

    Eigen::Matrix<double,6,6> C;
    C.setZero();
    C = C_rb + C_a;

    // std::cout<<C * twist_B<<std::endl;
    // std::cout<<"===================="<<std::endl;


    return C_rb * twist_B;
}

Sophus::Vector6d BLUEROV2_STATES::ImuDoNodelet::dynamics_D(
    const Sophus::Vector6d& twist_B
)
{
    Eigen::Matrix<double,1,6> D_diagonal;
    D_diagonal << 
        - Dl[0] - Dnl[0] * abs(twist_B(0)), 
        - Dl[1] - Dnl[1] * abs(twist_B(1)), 
        - Dl[2] - Dnl[2] * abs(twist_B(2)),
        - Dl[3] - Dnl[3] * abs(twist_B(3)), 
        - Dl[4] - Dnl[4] * abs(twist_B(4)), 
        - Dl[5] - Dnl[5] * abs(twist_B(5));

    Eigen::Matrix<double,6,6> D;
    D = D_diagonal.asDiagonal();

    return D * twist_B;
}

Sophus::Vector6d BLUEROV2_STATES::ImuDoNodelet::dynamics_Ma(
    const Sophus::Vector6d& imu_B
)
{
    Eigen::Vector3d g_W(0,0,-9.81);
    Eigen::Vector3d g_B = vehicle_SE3_world_gt.rotationMatrix().inverse() * g_W;
    Sophus::Vector6d v_dot_B = imu_B;

    v_dot_B.head(3) = v_dot_B.head(3) + g_B;

    // std::cout<<v_dot_B<<std::

    // std::cout<<v_dot_B<<std::endl;
    
    // std::cout<<"ma here"<<std::endl;
    // std::cout<<M_a * v_dot_B<<std::endl;
    // std::cout<<"==================="<<std::endl;

    return M_a * v_dot_B;

}

Sophus::Vector6d BLUEROV2_STATES::ImuDoNodelet::dynamics_g(
    const Eigen::Vector3d& euler
)
{
    return (
        Sophus::Vector6d()
        <<
        (mass * g_constant - bouyancy) * sin(euler(1)),
        -(mass * g_constant - bouyancy) * cos(euler(1)) * sin(euler(0)),
        -(mass * g_constant - bouyancy) * cos(euler(1)) * cos(euler(0)),
        ZG * mass * g_constant * cos(euler(1)) * sin(euler(0)),
        ZG * mass * g_constant * sin(euler(1)),
        0
    ).finished();
}

void BLUEROV2_STATES::ImuDoNodelet::thrusts_cb(
    const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg, 
    int index
)
{
    double input = msg->data;
    // ROS_INFO("Received input for thruster %d: %f", index, input);
    switch (index)
    {
        case 0:
            current_th(0) = input;
            break;
        case 1:
            current_th(1) = input;
            break;
        case 2:
            current_th(2) = input;
            break;
        case 3:
            current_th(3) = input;
            break;
        case 4:
            current_th(4) = input;
            break;
        case 5:
            current_th(5) = input;
            break;
        default:
            ROS_WARN("Invalid thruster index: %d", index);
            break;
    }
}