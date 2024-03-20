#include "bluerov2_states/ImuDo.h"

void BLUEROV2_STATES::ImuDoNodelet::DistRawMeas()
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

    std::cout<<imu_raw_B - vehicle_twist_body_gt<<std::endl;

    esti_dist.disturb.linear.x = delta_raw_B(0);
    esti_dist.disturb.linear.y = delta_raw_B(1);
    esti_dist.disturb.linear.z = delta_raw_B(2);

    esti_dist_pub.publish(esti_dist);

}

void BLUEROV2_STATES::ImuDoNodelet::dynamics_parameter_config()
{
    K << 0.7071067811847433, 0.7071067811847433, -0.7071067811919605, -0.7071067811919605, 0.0, 0.0,
       0.7071067811883519, -0.7071067811883519, 0.7071067811811348, -0.7071067811811348, 0.0, 0.0,
       0, 0, 0, 0, 1, 1,
       0.051265241636155506, -0.05126524163615552, 0.05126524163563227, -0.05126524163563227, -0.11050000000000001, 0.11050000000000003,
       -0.05126524163589389, -0.051265241635893896, 0.05126524163641713, 0.05126524163641713, -0.002499999999974481, -0.002499999999974481,
       0.16652364696949604, -0.16652364696949604, -0.17500892834341342, 0.17500892834341342, 0.0, 0.0;
    
    M_rb = (
        Sophus::Vector6d()
        <<
        mass, mass, mass, Ix, Iy, Iz
    ).finished().asDiagonal();
    
    M = M_values.asDiagonal();
    M_rb(0,4) = mass * ZG;
    M_rb(1,3) = - mass * ZG;
    M_rb(3,1) = - mass * ZG;
    M_rb(4,0) = mass * ZG;

    M_a = (
        Sophus::Vector6d()
        <<
        added_mass[0],
        added_mass[1],
        added_mass[2],
        added_mass[3],
        added_mass[4],
        added_mass[5]
    ).finished().asDiagonal();

    M = M_rb + M_a;

    invM = M.inverse();

    std::cout<<M<<std::endl<<std::endl;
    std::cout<<M_rb<<std::endl;

    // patty::Debug("test");
}

Sophus::Vector6d BLUEROV2_STATES::ImuDoNodelet::cal_system_wrench()
{
    return 
        dynamics_Tau(current_th)
        -
        dynamics_C(vehicle_twist_body_gt)
        -
        dynamics_D(vehicle_twist_body_gt)
        -
        dynamics_Ma(imu_raw_B)
        -
        dynamics_g(vehicle_Euler_gt);
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
    Eigen::Matrix<double,6,6> C;
    C<< 0, 0, 0, 0, mass * twist_B(2) + added_mass[2] * twist_B(2), - mass * twist_B(1) + added_mass[1] * twist_B(1),
        0, 0, 0, - mass * twist_B(2) - added_mass[2] * twist_B(2), 0, mass * twist_B(0) - added_mass[0] * twist_B(0),
        0, 0, 0,  mass * twist_B(1) - added_mass[1] * twist_B(1), - mass * twist_B(0) + added_mass[0] * twist_B(0), 0,
        0, mass * twist_B(2) - added_mass[2] * twist_B(2), - mass * twist_B(1) + added_mass[1] * twist_B(1), 0, Iz * twist_B(5)-added_mass[5]*twist_B(5), -Iy*twist_B(4)+added_mass[4]*twist_B(4),
        - mass * twist_B(2) + added_mass[2] * twist_B(2), 0, mass * twist_B(0) - added_mass[0] * twist_B(0), -Iz*twist_B(5)+added_mass[5]*twist_B(5), 0, Ix*twist_B(3)-added_mass[3]*twist_B(3),
        mass * twist_B(1) - added_mass[1] * twist_B(1), - mass * twist_B(0) + added_mass[0] * twist_B(0), 0, Iy*twist_B(4)-added_mass[4]*twist_B(4), -Ix*twist_B(3)+added_mass[3]*twist_B(3), 0;

    return C * twist_B;
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

    // std::cout<<"v_dot_B"<<std::endl;
    // std::cout<<v_dot_B<<std::endl;

    return v_dot_B;

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