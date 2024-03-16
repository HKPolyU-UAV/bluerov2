#include "bluerov2_dobmpc/bluerov2_ctrl.h"

// Initialize MPC
BLUEROV2_CTRL::BLUEROV2_CTRL(ros::NodeHandle& nh)
{
    ctrl_config(nh);
    communi_config(nh);
}

void BLUEROV2_CTRL::ctrl_config(ros::NodeHandle& nh)
{
    // read parameter
    nh.getParam("/bluerov2_ctrl_node/auto_yaw",AUTO_YAW);
    nh.getParam("/bluerov2_ctrl_node/compensate_d",COMPENSATE_D);
    nh.getParam("/bluerov2_ctrl_node/ctrller_type", ctrller_type);

    switch (ctrller_type)
    {
    case MPC:
        /* code */
        ROS_GREEN_STREAM("MPC IN CHARGE!");
        mpc_config(false, nh);
        break;

    case DOMPC:
        ROS_GREEN_STREAM("DOMPC IN CHARGE!");
        mpc_config(true, nh);
        break;
    
    case PID:
        /* code */
        ROS_GREEN_STREAM("PID IN CHARGE!");
        pid_config(nh);
        break;
    
    default:
        ROS_ERROR("CTRLLER TYPE WRONG!");
        break;
    }
}

void BLUEROV2_CTRL::communi_config(ros::NodeHandle& nh)
{
    // odom subsriber 
    // topic name should be written with yaml
    pose_sub = nh.subscribe<nav_msgs::Odometry>
                ("/bluerov2/pose_gt", 20, &BLUEROV2_CTRL::pose_cb, this);
    
    ref_sub = nh.subscribe<airo_message::BlueRefPreview>
                ("/ref_traj", 1, &BLUEROV2_CTRL::ref_cb, this);

    disturb_esti_sub = nh.subscribe<airo_message::Disturbance>
                ("/disturbance", 1, &BLUEROV2_CTRL::dist_cb, this);

    // ctrl pub
    thrust0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/0/input",20);
    thrust1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/1/input",20);
    thrust2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/2/input",20);
    thrust3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/3/input",20);
    thrust4_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/4/input",20);
    thrust5_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/5/input",20);
        
    // vis pub
    ref_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/mpc/reference",20);
    error_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/mpc/error",20);
    control_input0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/0",20);
    control_input1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/1",20);
    control_input2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/2",20);
    control_input3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/3",20);

    // timer
    mainspin_timer = nh.createTimer(
        ros::Duration(1.0 / 20.0),
        &BLUEROV2_CTRL::mainspin_cb,
        this
    );
}

void BLUEROV2_CTRL::pose_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
    vehicle_SE3_world = posemsg_to_SE3(odom->pose.pose);
    vehicle_twist_world = twistmsg_to_velo(odom->twist.twist);
    
    vehicle_twist_body.head(3) = 
        vehicle_SE3_world.rotationMatrix().inverse() * vehicle_twist_world.head(3);
    vehicle_twist_body.tail(3) = 
        Jacobi3dR(vehicle_SE3_world).inverse() * vehicle_twist_world.tail(3);

    vehicle_Euler = q2rpy(vehicle_SE3_world.unit_quaternion());
}

void BLUEROV2_CTRL::ref_cb(const airo_message::BlueRefPreview::ConstPtr& msg)
{
    is_start = true;
    got_path = true;
    ref_traj = *msg;

    if(
        ref_traj.preview.size() == 1 
        ||
        ctrller_type == PID
    )
    {
        ref_single_pt = ref_traj.preview[0];
    }
}

void BLUEROV2_CTRL::dist_cb(const airo_message::Disturbance::ConstPtr& msg)
{
    esti_disturb = *msg;
}

void BLUEROV2_CTRL::mainspin_cb(const ros::TimerEvent& e)
{
    if(!is_start)
        return;

    switch (ctrller_type)
    {
    case MPC:
        mpc_solve();
        break;
    
    case PID:
        pid_solve();
        break;

    default:
        break;
    }

    got_path = false;
}

void BLUEROV2_CTRL::set_current_yaw_for_ctrl()
{
    double psi = vehicle_Euler(2);

    // identify turning direction
    if (pre_yaw >= 0 && psi >=0)
    {
        yaw_diff = psi - pre_yaw;
    }


    else if (pre_yaw >= 0 && psi <0)
    {
        if (2*M_PI + psi - pre_yaw >= pre_yaw + abs(psi))
        {
            yaw_diff = -(pre_yaw + abs(psi));
        }
        else
        {
            yaw_diff = 2 * M_PI + psi - pre_yaw;
        }
    }
    else if (pre_yaw < 0 && psi >= 0)
    {
        if (2*M_PI-psi+pre_yaw >= abs(pre_yaw)+psi)
        {
            yaw_diff = abs(pre_yaw)+psi;
        }
        else
        {
            yaw_diff = -(2*M_PI-psi+pre_yaw);
        }
    }
    else
    {
        yaw_diff = psi - pre_yaw;
    }

    pre_yaw = psi;
    yaw_sum = yaw_sum + yaw_diff;
}

void BLUEROV2_CTRL::ctrl_allocate(const SolverOutput& u_out)
{
    thrust0.data=(-u_out.u0[0]+u_out.u0[1]+u_out.u0[3])/rotor_constant;
    thrust1.data=(-u_out.u0[0]-u_out.u0[1]-u_out.u0[3])/rotor_constant;
    thrust2.data=(u_out.u0[0]+u_out.u0[1]-u_out.u0[3])/rotor_constant;
    thrust3.data=(u_out.u0[0]-u_out.u0[1]+u_out.u0[3])/rotor_constant;
    thrust4.data=(-u_out.u0[2])/rotor_constant;
    thrust5.data=(-u_out.u0[2])/rotor_constant;
}























void BLUEROV2_CTRL::misc_pub()
{
    // change into form of (-pi, pi)
    if(sin(acados_in.yref[0][5]) >= 0)
        yaw_ref = fmod(
            acados_in.yref[0][5],
            M_PI
        );
    
    else
        yaw_ref = -M_PI + fmod(
            acados_in.yref[0][5],
            M_PI
        );


    // publish reference pose
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw_ref);
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat, quat_msg);
    ref_pose.pose.pose.position.x = acados_in.yref[0][0];
    ref_pose.pose.pose.position.y = acados_in.yref[0][1];
    ref_pose.pose.pose.position.z = acados_in.yref[0][2];
    ref_pose.pose.pose.orientation.x = quat_msg.x;
    ref_pose.pose.pose.orientation.y = quat_msg.y;
    ref_pose.pose.pose.orientation.z = quat_msg.z;
    ref_pose.pose.pose.orientation.w = quat_msg.w;
    ref_pose.header.stamp = ros::Time::now();
    ref_pose.header.frame_id = "odom_frame";
    ref_pose.child_frame_id = "base_link";
    ref_pose_pub.publish(ref_pose);

    // publish error pose
    tf2::Quaternion quat_error;
    yaw_error = yaw_sum - acados_in.yref[0][5];
    quat_error.setRPY(0, 0, yaw_error);
    geometry_msgs::Quaternion quat_error_msg;
    tf2::convert(quat_error, quat_error_msg);
    error_pose.pose.pose.position.x = acados_in.x0[0] - acados_in.yref[0][0];
    error_pose.pose.pose.position.y = acados_in.x0[1] - acados_in.yref[0][1];
    error_pose.pose.pose.position.z = acados_in.x0[2] - acados_in.yref[0][2];
    error_pose.pose.pose.orientation.x = quat_error_msg.x;
    error_pose.pose.pose.orientation.y = quat_error_msg.y;
    error_pose.pose.pose.orientation.z = quat_error_msg.z;
    error_pose.pose.pose.orientation.w = quat_error_msg.w;
    error_pose.header.stamp = ros::Time::now();
    error_pose.header.frame_id = "odom_frame";
    error_pose.child_frame_id = "base_link";

    error_pose_pub.publish(error_pose);

    // publish conrtrol input
    control_input0.data = acados_out.u0[0];
    control_input1.data = acados_out.u0[1];
    control_input2.data = acados_out.u0[2];
    control_input3.data = acados_out.u0[3];

    control_input0_pub.publish(control_input0);
    control_input1_pub.publish(control_input1);
    control_input2_pub.publish(control_input2);
    control_input3_pub.publish(control_input3);
}
