#include "bluerov2_dobmpc/bluerov2_dob_ctrl.h"

// Initialize MPC
BLUEROV2_DOB_CTRL::BLUEROV2_DOB_CTRL(ros::NodeHandle& nh)
{
    ctrl_config(nh);
    communi_config(nh);
}

void BLUEROV2_DOB_CTRL::ctrl_config(ros::NodeHandle& nh)
{
    // read parameter
    nh.getParam("/bluerov2_dob_node/auto_yaw",AUTO_YAW);
    nh.getParam("/bluerov2_dob_node/compensate_d",COMPENSATE_D);

    // Initialize MPC
    int create_status = 1;
    create_status = bluerov2_acados_create(mpc_capsule);
    if (create_status != 0){
        ROS_INFO_STREAM("acados_create() returned status " << create_status << ". Exiting." << std::endl);
        exit(1);
    }

    // propulsion matrix should be written with yaml
    K << 0.7071067811847433, 0.7071067811847433, -0.7071067811919605, -0.7071067811919605, 0.0, 0.0,
       0.7071067811883519, -0.7071067811883519, 0.7071067811811348, -0.7071067811811348, 0.0, 0.0,
       0, 0, 0, 0, 1, 1,
       0.051265241636155506, -0.05126524163615552, 0.05126524163563227, -0.05126524163563227, -0.11050000000000001, 0.11050000000000003,
       -0.05126524163589389, -0.051265241635893896, 0.05126524163641713, 0.05126524163641713, -0.002499999999974481, -0.002499999999974481,
       0.16652364696949604, -0.16652364696949604, -0.17500892834341342, 0.17500892834341342, 0.0, 0.0;
   
    // initialize
    for(unsigned int i=0; i < BLUEROV2_NU; i++) 
        acados_out.u0[i] = 0.0;
    for(unsigned int i=0; i < BLUEROV2_NX; i++) 
        acados_in.x0[i] = 0.0;

    is_start = false;
}

void BLUEROV2_DOB_CTRL::communi_config(ros::NodeHandle& nh)
{
    // odom subsriber 
    // topic name should be written with yaml
    pose_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/pose_gt", 20, &BLUEROV2_DOB_CTRL::pose_cb, this);
    // disturb_esti_sub = nh.subscribe<

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
        &BLUEROV2_DOB_CTRL::mainspin_cb,
        this
    );
}

void BLUEROV2_DOB_CTRL::pose_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
    is_start = true;

    vehicle_SE3_world = posemsg_to_SE3(odom->pose.pose);
    vehicle_twist_world = twistmsg_to_velo(odom->twist.twist);
    
    vehicle_twist_body.head(3) = 
        vehicle_SE3_world.rotationMatrix().inverse() * vehicle_twist_world.head(3);
    vehicle_twist_body.tail(3) = 
        Jacobi3d(vehicle_SE3_world).inverse() * vehicle_twist_world.tail(3);

    vehicle_Euler = q2rpy(vehicle_SE3_world.unit_quaternion());
}

void BLUEROV2_DOB_CTRL::ref_cb(const airo_message::ReferencePreview::ConstPtr& msg)
{
    

    
}

void BLUEROV2_DOB_CTRL::mainspin_cb(const ros::TimerEvent& e)
{
    solve();
}

// solve MPC
// input: current pose, reference, parameter
// output: thrust<0-5>
void BLUEROV2_DOB_CTRL::solve()
{
    set_mpc_initial_state();
    set_mpc_constraints();

    //now reference
    // change into form of (-pi, pi)
    if(sin(acados_in.yref[0][5]) >= 0)
    {
        yaw_ref = fmod(acados_in.yref[0][5],M_PI);
    }
    else{
        yaw_ref = -M_PI + fmod(acados_in.yref[0][5],M_PI);
    }

    // set reference!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // ref_cb(line_number); 

    for (unsigned int i = 0; i <= BLUEROV2_N; i++)
    {
        // set_ref
        ocp_nlp_cost_model_set(
            mpc_capsule->nlp_config, 
            mpc_capsule->nlp_dims, 
            mpc_capsule->nlp_in, 
            i, 
            "yref", 
            acados_in.yref[i]
        );
    }

    // Solve OCP
    acados_status = bluerov2_acados_solve(mpc_capsule);

    if (acados_status != 0){
        ROS_INFO_STREAM("acados returned status " << acados_status << std::endl);
    }

    acados_out.status = acados_status;
    acados_out.kkt_res = (double)mpc_capsule->nlp_out->inf_norm_res;

    ocp_nlp_get(mpc_capsule->nlp_config, mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);

    ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "u", (void *)acados_out.u0);
    
    thrust0.data=(-acados_out.u0[0]+acados_out.u0[1]+acados_out.u0[3])/rotor_constant;
    thrust1.data=(-acados_out.u0[0]-acados_out.u0[1]-acados_out.u0[3])/rotor_constant;
    thrust2.data=(acados_out.u0[0]+acados_out.u0[1]-acados_out.u0[3])/rotor_constant;
    thrust3.data=(acados_out.u0[0]-acados_out.u0[1]+acados_out.u0[3])/rotor_constant;
    thrust4.data=(-acados_out.u0[2])/rotor_constant;
    thrust5.data=(-acados_out.u0[2])/rotor_constant;
    
    thrust0_pub.publish(thrust0);
    thrust1_pub.publish(thrust1);
    thrust2_pub.publish(thrust2);
    thrust3_pub.publish(thrust3);
    thrust4_pub.publish(thrust4);
    thrust5_pub.publish(thrust5);

    misc_pub();
}

void BLUEROV2_DOB_CTRL::set_mpc_initial_state()
{
    // set initial states (current state)
    acados_in.x0[s_x] = vehicle_SE3_world.translation().x();
    acados_in.x0[s_y] = vehicle_SE3_world.translation().y();
    acados_in.x0[s_z] = vehicle_SE3_world.translation().z();

    acados_in.x0[s_phi] = vehicle_Euler(0);
    acados_in.x0[s_theta] = vehicle_Euler(1);
    acados_in.x0[s_psi] = yaw_sum;

    acados_in.x0[s_u] = vehicle_twist_body(0);
    acados_in.x0[s_v] = vehicle_twist_body(1);
    acados_in.x0[s_w] = vehicle_twist_body(2);

    acados_in.x0[s_p] = vehicle_twist_body(3);
    acados_in.x0[s_q] = vehicle_twist_body(4);
    acados_in.x0[s_r] = vehicle_twist_body(5);
}

void BLUEROV2_DOB_CTRL::set_mpc_constraints()
{
    ocp_nlp_constraints_model_set(
        mpc_capsule->nlp_config,
        mpc_capsule->nlp_dims,
        mpc_capsule->nlp_in, 
        0, 
        "lbx", 
        acados_in.x0
    );

    ocp_nlp_constraints_model_set(
        mpc_capsule->nlp_config,
        mpc_capsule->nlp_dims,
        mpc_capsule->nlp_in, 
        0, 
        "ubx", 
        acados_in.x0
    );

    // set parameters
    for (int i = 0; i < BLUEROV2_N+1; i++)
    {
        if(COMPENSATE_D == false){
            // sub
            // acados_param[i][0] = esti_disturb.disturb_x;
            // acados_param[i][1] = esti_disturb.disturb_y;
            // acados_param[i][2] = esti_disturb.disturb_x;
            // acados_param[i][3] = esti_disturb.disturb_x;
        }
        else if(COMPENSATE_D == true){

            //! what is this
            // acados_param[i][0] = esti_x(12)/compensate_coef;
            // acados_param[i][1] = esti_x(13)/compensate_coef;
            // acados_param[i][2] = esti_x(14)/rotor_constant;
            // acados_param[i][3] = esti_x(17)/rotor_constant;  
        }
        // added mass
        acados_param[i][4] = 1.7182;
        acados_param[i][5] = 0;
        acados_param[i][6] = 5.468;
        acados_param[i][7] = 0.4006;
        // linear d
        acados_param[i][8] = -11.7391;
        acados_param[i][9] = -20;
        acados_param[i][10] = -31.8678;
        acados_param[i][11] = -5;
        // nonlinear d
        acados_param[i][12] = -18.18;
        acados_param[i][13] = -21.66;
        acados_param[i][14] = -36.99;
        acados_param[i][15] = -1.55;

        bluerov2_acados_update_params(
            mpc_capsule,
            i,
            acados_param[i],
            BLUEROV2_NP
        );
    }
}

double BLUEROV2_DOB_CTRL::set_current_yaw_for_ctrl()
{
    double psi;

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

    return yaw_sum;
}

void BLUEROV2_DOB_CTRL::misc_pub()
{
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