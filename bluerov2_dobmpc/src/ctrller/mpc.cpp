#include "bluerov2_dobmpc/bluerov2_ctrl.h"


// solve MPC
// input: current pose, reference, parameter
// output: thrust<0-5>
void BLUEROV2_CTRL::mpc_config(
    bool hv_disrub,
    ros::NodeHandle& nh
)
{
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

    if(hv_disrub)
        disturb_esti_sub = nh.subscribe<airo_message::Disturbance>
                ("/disturbance", 1, &BLUEROV2_CTRL::dist_cb, this);
}

void BLUEROV2_CTRL::mpc_solve()
{
    set_mpc_initial_state();
    set_mpc_constraints();

    set_ref();

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
        patty::Debug("HOW THE FUCK?");
        std::cout<<"EXIT!"<<std::endl;
        return;
    }

    ROS_GREEN_STREAM("SUCCEED!");

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

    // std::cout<<"PASS TO ACADOS VALUE"<<std::endl;
    // for(auto what : acados_in.x0)
    // {
    //     std::cout<<what<<std::endl;
    // }
    // std::cout<<std::endl;
    // std::cout<<"ENDING PASS TO ACADOS VALUE"<<std::endl;
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
        if(COMPENSATE_D == false)
        {
            acados_param[i][0] = 0; //esti_disturb.disturb_x;
            acados_param[i][1] = 0; //esti_disturb.disturb_y;
            acados_param[i][2] = 0; //esti_disturb.disturb_x;
            acados_param[i][3] = 0; //esti_disturb.disturb_x;
        }
        else if(COMPENSATE_D == true)
        {
            acados_param[i][0] = esti_disturb.disturb.linear.x; //esti_x(12)/compensate_coef;
            acados_param[i][1] = esti_disturb.disturb.linear.y; //esti_x(13)/compensate_coef;
            acados_param[i][2] = esti_disturb.disturb.linear.z; //esti_x(14)/rotor_constant;
            acados_param[i][3] = esti_disturb.disturb.angular.z; //esti_x(17)/rotor_constant;  
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
    using namespace std;

    // change into form of (-pi, pi)
    if(sin(acados_in.yref[0][5]) >= 0)
    {
        yaw_ref = fmod(acados_in.yref[0][5],M_PI);
    }
    else{
        yaw_ref = -M_PI + fmod(acados_in.yref[0][5],M_PI);
    }

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
        patty::Debug("SIZE WRONG!");

    for(int i = 0 ; i <= BLUEROV2_N; i++)
        convert_refmsg_2_acados(i, ref_traj.preview[i]);
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

