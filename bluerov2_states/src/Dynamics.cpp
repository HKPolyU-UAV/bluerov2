#include "bluerov2_states/ImuDo.h"

void BLUEROV2_STATES::ImuDoNodelet::dynamics_parameter_config()
{


}

// void 

Eigen::MatrixXd BLUEROV2_STATES::ImuDoNodelet::dynamics_C(
    const Sophus::Vector6d& twist_B
)
{
    Eigen::Matrix<double,6,6> C;
    C<< 0, 0, 0, 0, mass*twist_B(2)+added_mass[2]*twist_B(2), -mass*twist_B(1)+added_mass[1]*twist_B(1),
        0, 0, 0, -mass*twist_B(2)-added_mass[2]*twist_B(2), 0, mass*twist_B(0)-added_mass[0]*twist_B(0),
        0, 0, 0, mass*twist_B(1)-added_mass[1]*twist_B(1), -mass*twist_B(0)+added_mass[0]*twist_B(0), 0,
        0, mass*twist_B(2)-added_mass[2]*twist_B(2), -mass*twist_B(1)+added_mass[1]*twist_B(1), 0, Iz*twist_B(5)-added_mass[5]*twist_B(5), -Iy*twist_B(4)+added_mass[4]*twist_B(4),
        -mass*twist_B(2)+added_mass[2]*twist_B(2), 0, mass*twist_B(0)-added_mass[0]*twist_B(0), -Iz*twist_B(5)+added_mass[5]*twist_B(5), 0, Ix*twist_B(3)-added_mass[3]*twist_B(3),
        mass*twist_B(1)-added_mass[1]*twist_B(1), -mass*twist_B(0)+added_mass[0]*twist_B(0), 0, Iy*twist_B(4)-added_mass[4]*twist_B(4), -Ix*twist_B(3)+added_mass[3]*twist_B(3), 0;
    return C;
}

Eigen::MatrixXd BLUEROV2_STATES::ImuDoNodelet::dynamics_D(
    const Sophus::Vector6d& twist_B
)
{
    Eigen::Matrix<double,1,6> D_diagonal;
    D_diagonal << -Dl[0]-Dnl[0]*abs(twist_B(0)), -Dl[1]-Dnl[1]*abs(twist_B(1)), -Dl[2]-Dnl[2]*abs(twist_B(2)),
                -Dl[3]-Dnl[3]*abs(twist_B(3)), -Dl[4]-Dnl[4]*abs(twist_B(4)), -Dl[5]-Dnl[5]*abs(twist_B(5));

    Eigen::Matrix<double,6,6> D;
    D = D_diagonal.asDiagonal();

}

Eigen::MatrixXd BLUEROV2_STATES::ImuDoNodelet::dynamics_g(
    const Eigen::Vector3d& euler
)
{
    Eigen::Matrix<double,6,1> g;

    g << bouyancy*sin(euler(1)),
        -bouyancy*cos(euler(1))*sin(euler(0)),
        -bouyancy*cos(euler(1))*cos(euler(0)),
        mass*ZG*g*cos(euler(1))*sin(euler(0)),
        mass*ZG*g*sin(euler(1)),
        0;

    return g;

}