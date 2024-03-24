#include "bluerov2_dobmpc/bluerov2_ctrl.h"

// Initialize MPC
BLUEROV2_CTRL::BLUEROV2_CTRL(ros::NodeHandle& nh)
{
    ctrl_config(nh);
    communi_config(nh);
    starting_pt_config(nh);
}

void BLUEROV2_CTRL::ctrl_config(ros::NodeHandle& nh)
{
    // read parameter
    nh.getParam("auto_yaw",AUTO_YAW);
    nh.getParam("ctrller_type", ctrller_type);

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

    ros::Rate rate(10.0);
    while(!is_start)
    {
        ROS_INFO("WAIT FOR POSE_CB!");
        ros::spinOnce();
        rate.sleep();
    }

    // ctrl pub
    thrust0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/0/input",20);
    thrust1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/1/input",20);
    thrust2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/2/input",20);
    thrust3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/3/input",20);
    thrust4_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/4/input",20);
    thrust5_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/5/input",20);
        
    // vis pub
    ref_point_pub = nh.advertise<geometry_msgs::PointStamped>("/bluerov2/mpc/reference",20);
    cur_point_pub = nh.advertise<geometry_msgs::PointStamped>("/bluerov2/mpc/state",20);
    error_point_pub = nh.advertise<geometry_msgs::PointStamped>("/bluerov2/mpc/error",20);
    error_abs_pub = nh.advertise<std_msgs::Float32>("/bluerov2/mpc/error_abs",20);

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

void BLUEROV2_CTRL::starting_pt_config(ros::NodeHandle& nh)
{
    bool setpt_or_not;
    nh.getParam("setpt", setpt_or_not);

    if(setpt_or_not)
    {
        XmlRpc::XmlRpcValue starting_pt_list;
        nh.getParam("/bluerov2_ctrl_node/starting_setpt", starting_pt_list);
        for(int i = 0; i < 3; i++) 
            starting_setpt(i) = static_cast<double>(starting_pt_list[i]);

        last_ref.ref_pos.x = starting_setpt.x();
        last_ref.ref_pos.y = starting_setpt.y();
        last_ref.ref_pos.z = starting_setpt.z();
        last_ref.ref_ang.x = 0;
        last_ref.ref_ang.y = 0;
        last_ref.ref_ang.z = 0;
    }
    else
    {
        last_ref.ref_pos.x = vehicle_SE3_world.translation().x();
        last_ref.ref_pos.y = vehicle_SE3_world.translation().y();
        last_ref.ref_pos.z = vehicle_SE3_world.translation().z();
        last_ref.ref_ang.x = vehicle_Euler.x();
        last_ref.ref_ang.y = vehicle_Euler.y();
        last_ref.ref_ang.z = vehicle_Euler.z();
    }
}

void BLUEROV2_CTRL::pose_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
    is_start = true;
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

void BLUEROV2_CTRL::mainspin_cb(const ros::TimerEvent& e)
{
    if(!is_start)
        return;

    if(!got_path)
    {
        ref_single_pt = last_ref;
    }
    else
        set_last_ref();

    switch (ctrller_type)
    {
    case MPC:
        ROS_GREEN_STREAM("MPC HERE");
        mpc_solve();
        break;

    case DOMPC:
        ROS_GREEN_STREAM("DOMPC HERE");
        mpc_solve();
        break;
    
    case PID:
        ROS_GREEN_STREAM("PID HERE");
        pid_solve();
        break;

    default:
        break;
    }

    misc_pub();
    got_path = false;
}

void BLUEROV2_CTRL::set_last_ref()
{
    std_msgs::Header header_temp;
    header_temp.frame_id = "world";
    header_temp.stamp = ros::Time::now();

    last_ref.ref_pos.x = vehicle_SE3_world.translation().x();
    last_ref.ref_pos.y = vehicle_SE3_world.translation().y();
    last_ref.ref_pos.z = vehicle_SE3_world.translation().z();

    last_ref.ref_ang.x = 0;
    last_ref.ref_ang.y = 0;
    last_ref.ref_ang.z = 0;
}

void BLUEROV2_CTRL::set_current_yaw_for_ctrl()
{
    const double psi = vehicle_Euler(2);

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
    using namespace std;
    
    // cout<<"within allocation"<<endl;
    for(auto what : u_out.u0)
    {
        // cout<<what<<endl;
        if(isnan(what))
        {
            ROS_ERROR("SOVLER WRONG!");
            return;
        }
    }

    // cout<<endl;

    thrust0.data=(-u_out.u0[0] + u_out.u0[1] + u_out.u0[3]) / rotor_constant;
    // cout<<thrust0.data<<endl;
    thrust1.data=(-u_out.u0[0] - u_out.u0[1] - u_out.u0[3]) / rotor_constant;
    // cout<<thrust1.data<<endl;
    thrust2.data=(u_out.u0[0] + u_out.u0[1] - u_out.u0[3]) / rotor_constant;
    // cout<<thrust2.data<<endl;
    thrust3.data=(u_out.u0[0] - u_out.u0[1] + u_out.u0[3]) / rotor_constant;
    // cout<<thrust3.data<<endl;
    thrust4.data=(-u_out.u0[2]) / rotor_constant;
    // cout<<thrust4.data<<endl;
    thrust5.data=(-u_out.u0[2]) / rotor_constant;
    // cout<<thrust5.data<<endl;

    // why thrust 4 and 5 (+-) are different when applying pid or mpc
}

void BLUEROV2_CTRL::misc_pub()
{
    std_msgs::Header stamp;
    stamp.frame_id = "world";
    stamp.stamp = ros::Time::now();

    // ref point
    geometry_msgs::PointStamped ref_point;
    ref_point.header = stamp;
    ref_point.point.x = current_ref.ref_pos.x;
    ref_point.point.y = current_ref.ref_pos.y;
    ref_point.point.z = current_ref.ref_pos.z;

    // cur point
    geometry_msgs::PointStamped current_point;
    current_point.header = stamp;
    current_point.point.x = acados_in.x0[0];
    current_point.point.y = acados_in.x0[1];
    current_point.point.z = acados_in.x0[2];

    // error point
    geometry_msgs::PointStamped error_point;
    error_point.header = stamp;
    error_point.point.x = current_point.point.x - ref_point.point.x;
    error_point.point.y = current_point.point.y - ref_point.point.y;
    error_point.point.z = current_point.point.z - ref_point.point.z;

    error_point_pub.publish(error_point);

    std_msgs::Float32 error_abs;
    error_abs.data = sqrt(
        pow(error_point.point.x,2) + 
        pow(error_point.point.y,2) +
        pow(error_point.point.z,2) 
    );
    error_abs_pub.publish(error_abs);

}
