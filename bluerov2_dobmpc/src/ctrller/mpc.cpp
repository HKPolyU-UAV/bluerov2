#include "bluerov2_dobmpc/bluerov2_ctrl.h"


// solve MPC
// input: current pose, reference, parameter
// output: thrust<0-5>
void BLUEROV2_CTRL::mpc_solve()
{
    set_mpc_initial_state();
    set_mpc_constraints();

    set_ref();

    return;

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
    
    ctrl_allocate(acados_out);
    
    thrust0_pub.publish(thrust0);
    thrust1_pub.publish(thrust1);
    thrust2_pub.publish(thrust2);
    thrust3_pub.publish(thrust3);
    thrust4_pub.publish(thrust4);
    thrust5_pub.publish(thrust5);
    
    misc_pub();
}

void BLUEROV2_CTRL::set_mpc_initial_state()
{
    set_current_yaw_for_ctrl();
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

void BLUEROV2_CTRL::set_mpc_constraints()
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
            acados_param[i][0] = 0; //esti_disturb.disturb_x;
            acados_param[i][1] = 0; //esti_disturb.disturb_y;
            acados_param[i][2] = 0; //esti_disturb.disturb_x;
            acados_param[i][3] = 0; //esti_disturb.disturb_x;
        }
        else if(COMPENSATE_D == true){

            //! what is this
            acados_param[i][0] = 0; //esti_x(12)/compensate_coef;
            acados_param[i][1] = 0; //esti_x(13)/compensate_coef;
            acados_param[i][2] = 0; //esti_x(14)/rotor_constant;
            acados_param[i][3] = 0; //esti_x(17)/rotor_constant;  
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


void BLUEROV2_CTRL::set_ref()
{
    if(!got_path)
    {
        ROS_RED_STREAM("NO PATH!");

        for(int i = 0; i <= BLUEROV2_N; i++)
            convert_refmsg_2_acados(i, last_ref);
        
        return;
    }

    ROS_GREEN_STREAM("GOT PATH!");

    // need to 
    if(ref_traj.preview.size() != BLUEROV2_N + 1)
        pc::pattyDebug("SIZE WRONG!");

    for(int i = 0 ; i <= BLUEROV2_N; i++)
        convert_refmsg_2_acados(i, ref_traj.preview[i]);
    
    set_last_ref();
}

void BLUEROV2_CTRL::set_last_ref()
{
    std_msgs::Header header_temp;
    header_temp.frame_id = "world";
    header_temp.stamp = ros::Time::now();

    last_ref.ref_pos.x = vehicle_SE3_world.translation().x();
    last_ref.ref_pos.y = vehicle_SE3_world.translation().y();
    last_ref.ref_pos.z = vehicle_SE3_world.translation().z();

    last_ref.ref_ang.x = vehicle_Euler.x();
    last_ref.ref_ang.y = vehicle_Euler.y();
    last_ref.ref_ang.z = vehicle_Euler.z();
}

void BLUEROV2_CTRL::convert_refmsg_2_acados(
    const int i,
    const airo_message::BlueRef ref_current
)
{
    acados_in.yref[i][0] = ref_current.ref_pos.x;
    acados_in.yref[i][1] = ref_current.ref_pos.y;
    acados_in.yref[i][2] = ref_current.ref_pos.z;

    acados_in.yref[i][3] = ref_current.ref_ang.x;
    acados_in.yref[i][4] = ref_current.ref_ang.y;
    acados_in.yref[i][5] = ref_current.ref_ang.z;

    acados_in.yref[i][6] = ref_current.ref_twist.linear.x;
    acados_in.yref[i][7] = ref_current.ref_twist.linear.y;
    acados_in.yref[i][8] = ref_current.ref_twist.linear.z;

    acados_in.yref[i][9] = ref_current.ref_twist.angular.x;
    acados_in.yref[i][10] = ref_current.ref_twist.angular.y;
    acados_in.yref[i][11] = ref_current.ref_twist.angular.z;
}

