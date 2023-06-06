#include <bluerov2_dobmpc/bluerov2_dobmpc.h>

// Initialize MPC
BLUEROV2_DOBMPC::BLUEROV2_DOBMPC()
{
    int create_status = 1;
    create_status = bluerov2_acados_create(mpc_capsule);
    if (create_status != 0){
        ROS_INFO_STREAM("acados_create() returned status " << create_status << ". Exiting." << std::endl);
        exit(1);
    }

    for(unsigned int i=0; i < BLUEROV2_NU; i++) acados_out.u0[i] = 0.0;
    for(unsigned int i=0; i < BLUEROV2_NX; i++) acados_in.x0[i] = 0.0;
}

// quaternion to euler angle
BLUEROV2_DOBMPC::Euler BLUEROV2_DOBMPC::q2rpy(const geometry_msgs::Quaternion& quaternion){
    tf::Quaternion tf_quaternion;
    Euler euler;
    tf::quaternionMsgToTF(quaternion,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(euler.phi, euler.theta, euler.psi);
    return euler;
}

//euler angle to quaternion
geometry_msgs::Quaternion BLUEROV2_DOBMPC::rpy2q(const Euler& euler){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(euler.phi, euler.theta, euler.psi);
    return quaternion;
}

// solve MPC
// input: current pose, reference, parameter
// output: thrust<0-5>
std::vector<uuv_gazebo_ros_plugins_msgs::FloatStamped> BLUEROV2_DOBMPC::solve(const nav_msgs::Odometry& pose, const bluerov2_dobmpc::Reference& ref, const SolverParam& param){
    for (int i = 0; i < BLUEROV2_N+1; ++i){
        acados_in.yref[i][0] = ref.ref_pose[i].position.x;
        acados_in.yref[i][1] = ref.ref_pose[i].position.y;
        acados_in.yref[i][2] = ref.ref_pose[i].position.z;
        acados_in.yref[i][3] = 0;
        acados_in.yref[i][4] = 0;
        ref_euler = q2rpy(ref.ref_pose[i].orientation);
        acados_in.yref[i][5] = ref_euler.psi;
        acados_in.yref[i][6] = ref.ref_twist[i].linear.x;
        acados_in.yref[i][7] = ref.ref_twist[i].linear.y;
        acados_in.yref[i][8] = ref.ref_twist[i].linear.z;
        acados_in.yref[i][9] = 0;
        acados_in.yref[i][10] = 0;
        acados_in.yref[i][11] = 0;
        acados_in.yref[i][12] = 0;
        acados_in.yref[i][13] = 0;
        acados_in.yref[i][14] = 0;
        acados_in.yref[i][15] = 0;
    }

    local_euler = q2rpy(pose.pose.pose.orientation);

    // solve discontinue yaw control
    if (abs(acados_in.yref[0][5] - local_euler.psi) > M_PI)
    {
        if (acados_in.yref[0][5] > local_euler.psi)
        {
            local_euler.psi = local_euler.psi + 2*M_PI;
        }
        else
        {
            local_euler.psi = local_euler.psi - 2*M_PI;
        }
    }

    acados_in.x0[x] = pose.pose.pose.position.x;
    acados_in.x0[y] = pose.pose.pose.position.y;
    acados_in.x0[z] = pose.pose.pose.position.z;
    acados_in.x0[phi] = local_euler.phi;
    acados_in.x0[theta] = local_euler.theta;
    acados_in.x0[psi] = local_euler.psi;
    acados_in.x0[u] = pose.twist.twist.linear.x;
    acados_in.x0[v] = pose.twist.twist.linear.y;
    acados_in.x0[w] = pose.twist.twist.linear.z;
    acados_in.x0[p] = pose.twist.twist.angular.x;
    acados_in.x0[q] = pose.twist.twist.angular.y;
    acados_in.x0[r] = pose.twist.twist.angular.z;

    acados_param[0] = param.disturbance_x;
    acados_param[1] = param.disturbance_y;
    acados_param[2] = param.disturbance_z;

    for (unsigned int i = 0; i <= BLUEROV2_N; i++){
        bluerov2_acados_update_params(mpc_capsule,i,acados_param,BLUEROV2_NP);
    }

    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "lbx", acados_in.x0);
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "ubx", acados_in.x0);
    for (unsigned int i = 0; i <= BLUEROV2_N; i++){
        ocp_nlp_cost_model_set(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_in, i, "yref", acados_in.yref[i]);
    }

    acados_status = bluerov2_acados_solve(mpc_capsule);

    if (acados_status != 0){
        ROS_INFO_STREAM("acados returned status " << acados_status << std::endl);
    }

    acados_out.status = acados_status;
    acados_out.kkt_res = (double)mpc_capsule->nlp_out->inf_norm_res;

    ocp_nlp_get(mpc_capsule->nlp_config, mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);

    ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "u", (void *)acados_out.u0);
    
    thrust0.data=(-acados_out.u0[0]+acados_out.u0[1]+acados_out.u0[3]);
    thrust1.data=(-acados_out.u0[0]-acados_out.u0[1]-acados_out.u0[3]);
    thrust2.data=(acados_out.u0[0]+acados_out.u0[1]-acados_out.u0[3]);
    thrust3.data=(acados_out.u0[0]-acados_out.u0[1]+acados_out.u0[3]);
    thrust4.data=(-acados_out.u0[2]);
    thrust5.data=(-acados_out.u0[2]);
    thrusts.push_back(thrust0);
    thrusts.push_back(thrust1);
    thrusts.push_back(thrust2);
    thrusts.push_back(thrust3);
    thrusts.push_back(thrust4);
    thrusts.push_back(thrust5);
    return thrusts;
}
