#include <bluerov2_dobmpc/bluerov2_dob.h>

// Initialize MPC
BLUEROV2_DOB::BLUEROV2_DOB(ros::NodeHandle& nh)
{
    // read parameter
    nh.getParam("/bluerov2_dob_node/auto_yaw",AUTO_YAW);
    nh.getParam("/bluerov2_dob_node/ref_traj", REF_TRAJ);
    nh.getParam("/bluerov2_dob_node/disturbance_x", solver_param.disturbance_x);
    nh.getParam("/bluerov2_dob_node/disturbance_y", solver_param.disturbance_y);
    nh.getParam("/bluerov2_dob_node/disturbance_z", solver_param.disturbance_z);
    
    // Pre-load the trajectory
    const char * c = REF_TRAJ.c_str();
	number_of_steps = readDataFromFile(c, trajectory);
	if (number_of_steps == 0){
		ROS_WARN("Cannot load CasADi optimal trajectory!");
	}
	else{
		ROS_INFO_STREAM("Number of steps of selected trajectory: " << number_of_steps << std::endl);
	}
    //std::cout<< "bluerov2_interface initialized"<<std::endl;

    // Initialize MPC
    int create_status = 1;
    create_status = bluerov2_acados_create(mpc_capsule);
    if (create_status != 0){
        ROS_INFO_STREAM("acados_create() returned status " << create_status << ". Exiting." << std::endl);
        exit(1);
    }

    // ros subsriber & publisher
    pose_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/pose_gt", 20, &BLUEROV2_DOB::pose_cb, this);
    thrust0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/0/input",20);
    thrust1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/1/input",20);
    thrust2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/2/input",20);
    thrust3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/3/input",20);
    thrust4_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/4/input",20);
    thrust5_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/5/input",20);    

    // initialize
    for(unsigned int i=0; i < BLUEROV2_NU; i++) acados_out.u0[i] = 0.0;
    for(unsigned int i=0; i < BLUEROV2_NX; i++) acados_in.x0[i] = 0.0;
}

void BLUEROV2_DOB::pose_cb(const nav_msgs::Odometry::ConstPtr& pose)
{
    // get linear position x, y, z
    pose_gt.pose.pose.position.x = pose->pose.pose.position.x;
    pose_gt.pose.pose.position.y = pose->pose.pose.position.y;
    pose_gt.pose.pose.position.z = pose->pose.pose.position.z;

    // get angle phi, theta, psi
    tf::quaternionMsgToTF(pose->pose.pose.orientation,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(local_euler.phi, local_euler.theta, local_euler.psi);

    // get linear velocity u, v, w
    pose_gt.twist.twist.linear.x = pose->twist.twist.linear.x;
    pose_gt.twist.twist.linear.y = pose->twist.twist.linear.y;
    pose_gt.twist.twist.linear.z = pose->twist.twist.linear.z;

    // get angular velocity p, q, r
    pose_gt.twist.twist.angular.x = pose->twist.twist.angular.x;
    pose_gt.twist.twist.angular.y = pose->twist.twist.angular.y;
    pose_gt.twist.twist.angular.z = pose->twist.twist.angular.z;
    }

// quaternion to euler angle
BLUEROV2_DOB::Euler BLUEROV2_DOB::q2rpy(const geometry_msgs::Quaternion& quaternion){
    tf::Quaternion tf_quaternion;
    Euler euler;
    tf::quaternionMsgToTF(quaternion,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(euler.phi, euler.theta, euler.psi);
    return euler;
}

// euler angle to quaternion
geometry_msgs::Quaternion BLUEROV2_DOB::rpy2q(const Euler& euler){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(euler.phi, euler.theta, euler.psi);
    return quaternion;
}

// read trajectory data
int BLUEROV2_DOB::readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data)
{
	std::ifstream file(fileName);
	std::string line;
	int number_of_lines = 0;

	if (file.is_open())
	{
        std::cout<<"file is open"<<std::endl;
		while(getline(file, line)){
			number_of_lines++;
			std::istringstream linestream( line );
			std::vector<double> linedata;
			double number;

			while( linestream >> number ){
				linedata.push_back( number );
			}
			data.push_back( linedata );
		}

		file.close();
	}
	else
	{
        std::cout<<"file not open"<<std::endl;
		return 0;
	}

	return number_of_lines;
}
void BLUEROV2_DOB::ref_cb(int line_to_read)
{
    //std::cout<< "start ref_cb"<<std::endl;
    if (BLUEROV2_N+line_to_read+1 <= number_of_steps)  // All ref points within the file
    {
        for (unsigned int i = 0; i <= BLUEROV2_N; i++)  // Fill all horizon with file data
        {
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[i+line_to_read][j];
            }
        }
        //std::cout<< "all ref points within the file"<<std::endl;
    }
    else if(line_to_read < number_of_steps)    // Part of ref points within the file
    {
        //std::cout<< "Part of ref points within the file"<<std::endl;
        for (unsigned int i = 0; i < number_of_steps-line_to_read; i++)    // Fill part of horizon with file data
        {
            
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[i+line_to_read][j];
            }
            
        }

        for (unsigned int i = number_of_steps-line_to_read; i <= BLUEROV2_N; i++)  // Fill the rest horizon with the last point
        {
            
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
            }
            
        }
    }
    else    // none of ref points within the file
    {
        //std::cout<<"none of ref points within the file"<<std::endl;
        for (unsigned int i = 0; i <= BLUEROV2_N; i++)  // Fill all horizon with the last point
        {
            
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
            }
            
        }
    }
    //std::cout<< "mpc_ref filled"<<std::endl;
    for (unsigned int i = 1; i <= BLUEROV2_N; i++)
    {
        if(abs(acados_in.yref[i][5]-acados_in.yref[i-1][5]) > M_PI)
        {
            if(acados_in.yref[i][5]<0)
            {
                acados_in.yref[i][5] = acados_in.yref[i][5]+2*M_PI;
            }
            else
            {
                acados_in.yref[i][5] = acados_in.yref[i][5]-2*M_PI;
            }
        }
    }
}

// solve MPC
// input: current pose, reference, parameter
// output: thrust<0-5>
void BLUEROV2_DOB::solve(){

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

    acados_in.x0[x] = pose_gt.pose.pose.position.x;
    acados_in.x0[y] = pose_gt.pose.pose.position.y;
    acados_in.x0[z] = pose_gt.pose.pose.position.z;
    acados_in.x0[phi] = local_euler.phi;
    acados_in.x0[theta] = local_euler.theta;
    acados_in.x0[psi] = local_euler.psi;
    acados_in.x0[u] = pose_gt.twist.twist.linear.x;
    acados_in.x0[v] = pose_gt.twist.twist.linear.y;
    acados_in.x0[w] = pose_gt.twist.twist.linear.z;
    acados_in.x0[p] = pose_gt.twist.twist.angular.x;
    acados_in.x0[q] = pose_gt.twist.twist.angular.y;
    acados_in.x0[r] = pose_gt.twist.twist.angular.z;

    acados_param[0] = solver_param.disturbance_x;
    acados_param[1] = solver_param.disturbance_y;
    acados_param[2] = solver_param.disturbance_z;

    for (unsigned int i = 0; i <= BLUEROV2_N; i++){
        bluerov2_acados_update_params(mpc_capsule,i,acados_param,BLUEROV2_NP);
    }

    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "lbx", acados_in.x0);
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "ubx", acados_in.x0);
    
    ref_cb(line_number); 
    line_number++;
    
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
    
    thrust0_pub.publish(thrust0);
    thrust1_pub.publish(thrust1);
    thrust2_pub.publish(thrust2);
    thrust3_pub.publish(thrust3);
    thrust4_pub.publish(thrust4);
    thrust5_pub.publish(thrust5);

    // print reference, current pose, control inputs, thrusts...
    if(cout_counter > 2){ //reduce cout rate
        std::cout << "------------------------------------------------------------------------------------------------" << std::endl;
        std::cout << "x_ref:    " << acados_in.yref[0][0] << "\ty_ref:   " << acados_in.yref[0][1] << "\tz_ref:    " << acados_in.yref[0][2] << "\tyaw_ref:    " << acados_in.yref[0][5] << std::endl;
        std::cout << "x_gt:     " << acados_in.x0[0] << "\ty_gt:     " << acados_in.x0[1] << "\tz_gt:     " << acados_in.x0[2] << "\tyaw_gt:     " << local_euler.psi << std::endl;
        std::cout << "roll_gt:        " << acados_in.x0[3] << "\t\tpitch_gt:        " << acados_in.x0[4] << std::endl;
        std::cout << "u1    : " << acados_out.u0[0] << "\tu2:    " << acados_out.u0[1] << "\tu3:    " << acados_out.u0[2] << "\tu4:    " << acados_out.u0[3] << std::endl;
        std::cout << "t0:  " << thrust0.data << "\tt1:  " << thrust1.data << "\tt2:  " << thrust2.data << "\tt3:  " << thrust3.data << "\tt4:  " << thrust4.data << "\tt5:  " << thrust5.data << std::endl;
        std::cout << "solve_time: "<< acados_out.cpu_time << "\tkkt_res: " << acados_out.kkt_res << "\tacados_status: " << acados_out.status << std::endl;
        std::cout << "ros_time:   " << std::fixed << ros::Time::now().toSec() << std::endl;
        std::cout << "------------------------------------------------------------------------------------------------" << std::endl;
        cout_counter = 0;
    }
    else{
        cout_counter++;
        }

    
}
/*
int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluerov2_dob");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    BLUEROV2_DOB br(nh);
    //std::cout<< "br constructed"<<std::endl;
    while(ros::ok()){
        //std::cout<< "ros::ok, go to run"<<std::endl;
        br.solve();
        //std::cout<< "run once"<<std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
*/