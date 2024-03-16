#include "bluerov2_dobmpc/bluerov2_ctrl.h"

void BLUEROV2_CTRL::pid_solve()
{
    // set_pid_ref();

    pid_4D(
        get_pid_ref(),
        get_pid_pose()
    );

    thrust0_pub.publish(thrust0);
    thrust1_pub.publish(thrust1);
    thrust2_pub.publish(thrust2);
    thrust3_pub.publish(thrust3);
    thrust4_pub.publish(thrust4);
    thrust5_pub.publish(thrust5);

    misc_pub();
}

void BLUEROV2_CTRL::pid_4D(
    const Sophus::Vector4d& Ref,
    const Sophus::Vector4d& Pose
)
{
    Sophus::Vector4d U;

    Error = Ref - Pose;

    Integral += Error * dt;

    Derivative = (Error - prevError) / dt;

    U = Kp.array() * Error.array() 
        + Ki.array() + Integral.array() 
        + Kd.array() * Derivative.array();

    pid_out.u0[0] = U(0);
    pid_out.u0[1] = U(1);
    pid_out.u0[2] = U(2);
    pid_out.u0[3] = U(3);

};

Sophus::Vector4d BLUEROV2_CTRL::get_pid_ref()
{
    return (
        Sophus::Vector4d() << 
        ref_single_pt.ref_pos.x,
        ref_single_pt.ref_pos.y,
        ref_single_pt.ref_pos.z,
        ref_single_pt.ref_ang.z
    ).finished();
}

Sophus::Vector4d BLUEROV2_CTRL::get_pid_pose()
{
    set_current_yaw_for_ctrl();

    return (
        Sophus::Vector4d() <<
        vehicle_SE3_world.translation().x(),
        vehicle_SE3_world.translation().y(),
        vehicle_SE3_world.translation().z(),
        yaw_sum
    ).finished();
}
