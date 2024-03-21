#include "bluerov2_dobmpc/bluerov2_ctrl.h"
#include <cstdlib>

/*
    Unusable at this moment: Need to write a cascaded controller.
*/


void BLUEROV2_CTRL::pid_config(ros::NodeHandle& nh)
{
    using namespace std;
    control_input_pub = nh.advertise<geometry_msgs::Wrench>
                ("/bluerov2/thruster_manager/input",1);

    XmlRpc::XmlRpcValue Kp_list, Ki_list, Kd_list;

    nh.getParam("/bluerov2_ctrl_node/Kp_list", Kp_list);
    nh.getParam("/bluerov2_ctrl_node/Ki_list", Ki_list);
    nh.getParam("/bluerov2_ctrl_node/Kd_list", Kd_list);

    std::ostringstream ostr;
    std::istringstream istr;

    for (int i = 0; i < 4; i++) {
        Kp(i) = static_cast<double>(Kp_list[i]);
        Ki(i) = static_cast<double>(Ki_list[i]);
        Kd(i) = static_cast<double>(Kd_list[i]);
    }


    Error.setZero();
    Derivative.setZero();
    Integral.setZero();
    prevError.setZero();
}

void BLUEROV2_CTRL::pid_solve()
{
    using namespace std;

    Sophus::Vector4d ref = get_pid_ref();
    Sophus::Vector4d pose = get_pid_pose();

    // cout<<"ref"<<endl<<endl;
    // cout<<ref<<endl<<endl;
    // cout<<"pose"<<endl<<endl;
    // cout<<pose<<endl<<endl;
    
    pid_4D(
        ref,
        pose
    );

    control_input_to_thrust.force.x = pid_out.u0[0];
    control_input_to_thrust.force.y = pid_out.u0[1];
    control_input_to_thrust.force.z = pid_out.u0[2];

    control_input_to_thrust.torque.x = 0;
    control_input_to_thrust.torque.y = 0;
    control_input_to_thrust.torque.z = 0;

    control_input_pub.publish(control_input_to_thrust);

    misc_pub();
}

void BLUEROV2_CTRL::pid_4D(
    const Sophus::Vector4d& Ref,
    const Sophus::Vector4d& Pose
)
{
    using namespace std;

    Sophus::Vector4d U_I, U_B;

    Error = Ref - Pose;

    cout<<"ERROR:"<<endl;
    cout<<Error<<endl<<endl;

    Integral += Error * dt;

    Derivative = (Error - prevError) / dt;

    prevError = Error;

    U_I = Kp.array() * Error.array() 
        + Ki.array() * Integral.array() 
        + Kd.array() * Derivative.array();

    U_B.head(3) = 
        vehicle_SE3_world.rotationMatrix().inverse() 
        // * vehicle_SE3_world.rotationMatrix()
        * U_I.head(3);

    pid_out.u0[0] = U_B(0);
    pid_out.u0[1] = U_B(1);
    pid_out.u0[2] = U_B(2);
    pid_out.u0[3] = 0;//U(3);

    cout<<"within PID"<<endl;
    for(auto what:pid_out.u0)
    {
        cout<<what<<endl;
    }
    cout<<endl;

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
