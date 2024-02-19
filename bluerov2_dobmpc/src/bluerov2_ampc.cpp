#include <bluerov2_dobmpc/bluerov2_ampc.h>

// Initialize MPC
BLUEROV2_AMPC::BLUEROV2_AMPC(ros::NodeHandle& nh)
{
    // read parameter
    nh.getParam("/bluerov2_ampc_node/auto_yaw",AUTO_YAW);
    nh.getParam("/bluerov2_ampc_node/read_wrench",READ_WRENCH);
    nh.getParam("/bluerov2_ampc_node/compensate_d",COMPENSATE_D);
    nh.getParam("/bluerov2_ampc_node/ref_traj", REF_TRAJ);
    nh.getParam("/bluerov2_ampc_node/applied_forcex", WRENCH_FX);
    nh.getParam("/bluerov2_ampc_node/applied_forcey", WRENCH_FY);
    nh.getParam("/bluerov2_ampc_node/applied_forcez", WRENCH_FZ);
    nh.getParam("/bluerov2_ampc_node/applied_torquez", WRENCH_TZ);
    
    // Pre-load the trajectory
    const char * c = REF_TRAJ.c_str();
	number_of_steps = readDataFromFile(c, trajectory);
	if (number_of_steps == 0){
		ROS_WARN("Cannot load CasADi optimal trajectory!");
	}
	else{
		ROS_INFO_STREAM("Number of steps of selected trajectory: " << number_of_steps << std::endl);
	}

    // Initialize MPC
    int create_status = 1;
    create_status = bluerov2_acados_create(mpc_capsule);
    if (create_status != 0){
        ROS_INFO_STREAM("acados_create() returned status " << create_status << ". Exiting." << std::endl);
        exit(1);
    }

    // Initialize EKF
    M_values << mass + added_mass[0], mass + added_mass[1], mass + added_mass[2], Ix + added_mass[3], Iy + added_mass[4], Iz + added_mass[5];
    M = M_values.asDiagonal();
    M(0,4) = mass*ZG;
    M(1,3) = -mass*ZG;
    M(3,1) = -mass*ZG;
    M(4,0) = mass*ZG;
    invM = M.inverse();

    // Dl_values << 0, -20, -31.8678, -25, -44.9085, -5;
    Dl_values << 0, 0, 0, 0, 0, 0;
    Dl = Dl_values.asDiagonal();

    K << 0.7071067811847433, 0.7071067811847433, -0.7071067811919605, -0.7071067811919605, 0.0, 0.0,
       0.7071067811883519, -0.7071067811883519, 0.7071067811811348, -0.7071067811811348, 0.0, 0.0,
       0, 0, 0, 0, 1, 1,
       0.051265241636155506, -0.05126524163615552, 0.05126524163563227, -0.05126524163563227, -0.11050000000000001, 0.11050000000000003,
       -0.05126524163589389, -0.051265241635893896, 0.05126524163641713, 0.05126524163641713, -0.002499999999974481, -0.002499999999974481,
       0.16652364696949604, -0.16652364696949604, -0.17500892834341342, 0.17500892834341342, 0.0, 0.0;
       
    Q_cov << pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,
            pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),
            pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2);
    noise_Q= Q_cov.asDiagonal();
    
    esti_x << 0,0,-20,0,0,0,0,0,0,0,0,0,6,6,6,0,0,0;
    esti_P = P0;

    // Initialize RLS-FF
    RLSX_P = MatrixXd::Identity(numParams, numParams);
    RLSY_P = MatrixXd::Identity(numParams, numParams);
    RLSZ_P = MatrixXd::Identity(numParams, numParams);
    RLSK_P = MatrixXd::Identity(numParams, numParams);
    RLSM_P = MatrixXd::Identity(numParams, numParams);
    RLSN_P = MatrixXd::Identity(numParams, numParams);
    lambda = 0.98;
    lambda_X = 0.9;
    lambda_Y = 0.9;
    lambda_Z = 0.9;
    lambda_N = 0.9;
    theta_X = VectorXd::Zero(numParams);
    theta_Y = VectorXd::Zero(numParams);
    theta_Z = VectorXd::Zero(numParams);
    // theta_K = VectorXd::Zero(numParams);
    // theta_M = VectorXd::Zero(numParams);
    theta_N = VectorXd::Zero(numParams);

    // Initialize body wrench force
    applied_wrench.fx = 0.0;
    applied_wrench.fy = 0.0;
    applied_wrench.fz = 0.0;
    applied_wrench.tx = 0.0;
    applied_wrench.ty = 0.0;
    applied_wrench.tz = 0.0;

    // ros subsriber & publisher
    pose_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/pose_gt", 20, &BLUEROV2_AMPC::pose_cb, this);
    thrust0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/0/input",20);
    thrust1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/1/input",20);
    thrust2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/2/input",20);
    thrust3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/3/input",20);
    thrust4_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/4/input",20);
    thrust5_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/5/input",20);
    ref_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/mpc/reference",20);
    error_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/mpc/error",20);
    control_input0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/0",20);
    control_input1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/1",20);
    control_input2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/2",20);
    control_input3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/3",20);    
    esti_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/ekf/pose",20);
    esti_disturbance_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/ekf/disturbance",20);
    applied_disturbance_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/applied_disturbance",20);
    esti_added_mass_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/systemID/added_mass",20);
    esti_damping_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/systemID/linear_damping",20);
    esti_Ndamping_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/systemID/nonlinear_damping",20);
    esti_env_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/systemID/environmental_disturbance",20);
    subscribers.resize(6);
    for (int i = 0; i < 6; i++)
    {
        std::string topic = "/bluerov2/thrusters/" + std::to_string(i) + "/thrust";
        subscribers[i] = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>(topic, 20, boost::bind(&BLUEROV2_AMPC::thrusts_cb, this, _1, i));
    }
    client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    // imu_sub = nh.subscribe<sensor_msgs::Imu>("/bluerov2/imu", 20, &BLUEROV2_AMPC::imu_cb, this);
    
    // initialize
    for(unsigned int i=0; i < BLUEROV2_NU; i++) acados_out.u0[i] = 0.0;
    for(unsigned int i=0; i < BLUEROV2_NX; i++) acados_in.x0[i] = 0.0;
    is_start = false;
}

void BLUEROV2_AMPC::pose_cb(const nav_msgs::Odometry::ConstPtr& pose)
{
    is_start = true;
    // get linear position x, y, z
    local_pos.x = pose->pose.pose.position.x;
    local_pos.y = pose->pose.pose.position.y;
    local_pos.z = pose->pose.pose.position.z;

    // get angle phi, theta, psi
    tf::quaternionMsgToTF(pose->pose.pose.orientation,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(local_euler.phi, local_euler.theta, local_euler.psi);

    // get linear velocity u, v, w
    local_pos.u = pose->twist.twist.linear.x;
    local_pos.v = pose->twist.twist.linear.y;
    local_pos.w = pose->twist.twist.linear.z;

    // get angular velocity p, q, r
    local_pos.p = pose->twist.twist.angular.x;
    local_pos.q = pose->twist.twist.angular.y;
    local_pos.r = pose->twist.twist.angular.z;

    // inertial frame velocity to body frame
    Matrix<double,3,1> v_linear_inertial;
    Matrix<double,3,1> v_angular_inertial;

    v_linear_inertial << local_pos.u, local_pos.v, local_pos.w;
    v_angular_inertial << local_pos.p, local_pos.q, local_pos.r;

    R_ib << cos(local_euler.psi)*cos(local_euler.theta), -sin(local_euler.psi)*cos(local_euler.phi)+cos(local_euler.psi)*sin(local_euler.theta)*sin(local_euler.phi), sin(local_euler.psi)*sin(local_euler.phi)+cos(local_euler.psi)*cos(local_euler.phi)*sin(local_euler.theta),
            sin(local_euler.psi)*cos(local_euler.theta), cos(local_euler.psi)*cos(local_euler.phi)+sin(local_euler.phi)*sin(local_euler.theta)*sin(local_euler.psi), -cos(local_euler.psi)*sin(local_euler.phi)+sin(local_euler.theta)*sin(local_euler.psi)*cos(local_euler.phi),
            -sin(local_euler.theta), cos(local_euler.theta)*sin(local_euler.phi), cos(local_euler.theta)*cos(local_euler.phi);
    T_ib << 1, sin(local_euler.psi)*sin(local_euler.theta)/cos(local_euler.theta), cos(local_euler.phi)*sin(local_euler.theta)/cos(local_euler.theta),
            0, cos(local_euler.phi), sin(local_euler.phi),
            0, sin(local_euler.phi)/cos(local_euler.theta), cos(local_euler.phi)/cos(local_euler.theta);
    v_linear_body = R_ib.inverse()*v_linear_inertial;
    v_angular_body = T_ib.inverse()*v_angular_inertial;

    body_acc.x = (v_linear_body[0]-pre_body_pos.u)/dt;
    body_acc.y = (v_linear_body[1]-pre_body_pos.v)/dt;
    body_acc.z = (v_linear_body[2]-pre_body_pos.w)/dt;
    body_acc.phi = (v_angular_body[0]-pre_body_pos.p)/dt;
    body_acc.theta = (v_angular_body[1]-pre_body_pos.q)/dt;
    body_acc.psi = (v_angular_body[2]-pre_body_pos.r)/dt;

    pre_body_pos.u = v_linear_body[0];
    pre_body_pos.v = v_linear_body[1];
    pre_body_pos.w = v_linear_body[2];
    pre_body_pos.p = v_angular_body[0];
    pre_body_pos.q = v_angular_body[1];
    pre_body_pos.r = v_angular_body[2];

    Matrix<double,3,1> compensate_f_inertial;
    Matrix<double,3,1> compensate_f_body;
    compensate_f_inertial << 20,0,0;
    compensate_f_body = R_ib.inverse()*compensate_f_inertial;


    }

// quaternion to euler angle
BLUEROV2_AMPC::Euler BLUEROV2_AMPC::q2rpy(const geometry_msgs::Quaternion& quaternion){
    tf::Quaternion tf_quaternion;
    Euler euler;
    tf::quaternionMsgToTF(quaternion,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(euler.phi, euler.theta, euler.psi);
    return euler;
}

// euler angle to quaternion
geometry_msgs::Quaternion BLUEROV2_AMPC::rpy2q(const Euler& euler){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(euler.phi, euler.theta, euler.psi);
    return quaternion;
}

// read trajectory data
int BLUEROV2_AMPC::readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data)
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
void BLUEROV2_AMPC::ref_cb(int line_to_read)
{
    if (BLUEROV2_N+line_to_read+1 <= number_of_steps)  // All ref points within the file
    {
        for (unsigned int i = 0; i <= BLUEROV2_N; i++)  // Fill all horizon with file data
        {
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[i+line_to_read][j];
            }
        }
    }
    else if(line_to_read < number_of_steps)    // Part of ref points within the file
    {
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
        for (unsigned int i = 0; i <= BLUEROV2_N; i++)  // Fill all horizon with the last point
        {
            
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
            }
            
        }
    }
    
}

// solve MPC
// input: current pose, reference, parameter
// output: thrust<0-5>
void BLUEROV2_AMPC::solve(){
    // identify turning direction
    if (pre_yaw >= 0 && local_euler.psi >=0)
    {
        yaw_diff = local_euler.psi - pre_yaw;
    }
    else if (pre_yaw >= 0 && local_euler.psi <0)
    {
        if (2*M_PI+local_euler.psi-pre_yaw >= pre_yaw+abs(local_euler.psi))
        {
            yaw_diff = -(pre_yaw + abs(local_euler.psi));
        }
        else
        {
            yaw_diff = 2 * M_PI + local_euler.psi - pre_yaw;
        }
    }
    else if (pre_yaw < 0 && local_euler.psi >= 0)
    {
        if (2*M_PI-local_euler.psi+pre_yaw >= abs(pre_yaw)+local_euler.psi)
        {
            yaw_diff = abs(pre_yaw)+local_euler.psi;
        }
        else
        {
            yaw_diff = -(2*M_PI-local_euler.psi+pre_yaw);
        }
    }
    else
    {
        yaw_diff = local_euler.psi - pre_yaw;
    }

    yaw_sum = yaw_sum + yaw_diff;
    pre_yaw = local_euler.psi;

    // set initial states
    acados_in.x0[x] = local_pos.x;
    acados_in.x0[y] = local_pos.y;
    acados_in.x0[z] = local_pos.z;
    acados_in.x0[phi] = local_euler.phi;
    acados_in.x0[theta] = local_euler.theta;
    acados_in.x0[psi] = yaw_sum;
    acados_in.x0[u] = v_linear_body[0];
    acados_in.x0[v] = v_linear_body[1];
    acados_in.x0[w] = v_linear_body[2];
    acados_in.x0[p] = v_angular_body[0];
    acados_in.x0[q] = v_angular_body[1];
    acados_in.x0[r] = v_angular_body[2];
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "lbx", acados_in.x0);
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "ubx", acados_in.x0);

    // set disturbance parameters
    for (int i = 0; i < BLUEROV2_N+1; i++)
    {
        if(COMPENSATE_D == false){
            acados_param[i][0] = 0;
            acados_param[i][1] = 0;
            acados_param[i][2] = 0;
            acados_param[i][3] = 0;
        }
        else if(COMPENSATE_D == true){
            acados_param[i][0] = theta_X(2)/compensate_coef;
            acados_param[i][1] = theta_Y(2)/compensate_coef;
            acados_param[i][2] = theta_Z(2)/rotor_constant;
            acados_param[i][3] = theta_N(2)/rotor_constant;
        // added mass
        acados_param[i][4] = 1.7182;
        acados_param[i][5] = 0;
        acados_param[i][6] = 5.468;
        acados_param[i][7] = 0.4006;
        // acados_param[i][4] = theta_X(0);
        // acados_param[i][5] = theta_Y(0);
        // acados_param[i][6] = theta_Z(0);
        // acados_param[i][7] = theta_N(0);

        // linear d
        acados_param[i][8] = -11.7391;
        acados_param[i][9] = -20;
        acados_param[i][10] = -31.8678;
        acados_param[i][11] = -5;
        // acados_param[i][8] = theta_X(1);
        // acados_param[i][9] = theta_Y(1);
        // acados_param[i][10] = theta_Z(1);
        // acados_param[i][11] = theta_N(1);

        // nonlinear d
        acados_param[i][12] = -18.18;
        acados_param[i][13] = -21.66;
        acados_param[i][14] = -36.99;
        acados_param[i][15] = -1.55;
        // acados_param[i][12] = theta_X(3);
        // acados_param[i][13] = theta_Y(3);
        // acados_param[i][14] = theta_Z(3);
        // acados_param[i][15] = theta_N(3);
        }
        bluerov2_acados_update_params(mpc_capsule,i,acados_param[i],BLUEROV2_NP);
    }

    // change into form of (-pi, pi)
    if(sin(acados_in.yref[0][5]) >= 0)
    {
        yaw_ref = fmod(acados_in.yref[0][5],M_PI);
    }
    else{
        yaw_ref = -M_PI + fmod(acados_in.yref[0][5],M_PI);
    }

    // set reference
    ref_cb(line_number); 
    line_number++;
    for (unsigned int i = 0; i <= BLUEROV2_N; i++){
        ocp_nlp_cost_model_set(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_in, i, "yref", acados_in.yref[i]);
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

// void BLUEROV2_AMPC::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
// {
//     // get linear position x, y, z
//     local_acc.x = round(msg->linear_acceleration.x*10000)/10000;
//     local_acc.y = round(msg->linear_acceleration.y*10000)/10000;
//     local_acc.z = round(msg->linear_acceleration.z*10000)/10000-g;
    
// }

void BLUEROV2_AMPC::thrusts_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg, int index)
{
    double input = msg->data;
    //ROS_INFO("Received input for thruster %d: %f", index, input);
    switch (index)
    {
        case 0:
            current_t.t0 = input;
            break;
        case 1:
            current_t.t1 = input;
            break;
        case 2:
            current_t.t2 = input;
            break;
        case 3:
            current_t.t3 = input;
            break;
        case 4:
            current_t.t4 = input;
            break;
        case 5:
            current_t.t5 = input;
            break;
        default:
            ROS_WARN("Invalid thruster index: %d", index);
            break;
    }
}

// Define EKF function
// inputs: current state estimate x, current covariance estimate P, input u, measurement y, 
//        process noise covariance Q, measuremnt noise covariance R
void BLUEROV2_AMPC::EKF()
{
    // get input u and measuremnet y
    meas_u << current_t.t0, current_t.t1, current_t.t2, current_t.t3, current_t.t4, current_t.t5;
    Matrix<double,6,1> tau;
    tau = K*meas_u;
    meas_y << local_pos.x, local_pos.y, local_pos.z, local_euler.phi, local_euler.theta, local_euler.psi,
            v_linear_body[0], v_linear_body[1], v_linear_body[2], v_angular_body[0], v_angular_body[1], v_angular_body[2],
            tau(0),tau(1),tau(2),tau(3),tau(4),tau(5);
    
    // Define Jacobian matrices of system dynamics and measurement model
    Matrix<double,18,18> F;     // Jacobian of system dynamics
    Matrix<double,18,18> H;     // Jacobian of measurement model

    // Define Kalman gain matrix
    Matrix<double,18,18> Kal;

    // Define prediction and update steps
    Matrix<double,18,1> x_pred;     // predicted state
    Matrix<double,18,18> P_pred;    // predicted covariance
    Matrix<double,18,1> y_pred;     // predicted measurement
    Matrix<double,18,1> y_err;      // measurement error
    
    // Prediction step: estimate state and covariance at time k+1|k
    F = compute_jacobian_F(esti_x, meas_u);             // compute Jacobian of system dynamics at current state and input
    x_pred = RK4(esti_x, meas_u);                       // predict state at time k+1|k
    // dx = f(esti_x, meas_u);                             // acceleration
    P_pred = F * esti_P * F.transpose() + noise_Q;      // predict covariance at time k+1|k
    
    // Update step: correct state and covariance using measurement at time k+1
    H = compute_jacobian_H(x_pred);                         // compute Jacobian of measurement model at predicted state
    y_pred = h(x_pred);                                     // predict measurement at time k+1
    y_err = meas_y - y_pred;                                // compute measurement error
    Kal = P_pred * H.transpose() * (H * P_pred * H.transpose() + noise_R).inverse();    // compute Kalman gain
    esti_x = x_pred + Kal * y_err;                          // correct state estimate
    esti_P = (MatrixXd::Identity(n, n) - Kal * H) * P_pred * (MatrixXd::Identity(n, n) - Kal * H).transpose() + Kal*noise_R*Kal.transpose(); // correct covariance estimate
    
    // body frame disturbance to inertial frame
    wf_disturbance << (cos(meas_y(5))*cos(meas_y(4)))*esti_x(12) + (-sin(meas_y(5))*cos(meas_y(3))+cos(meas_y(5))*sin(meas_y(4))*sin(meas_y(3)))*esti_x(13) + (sin(meas_y(5))*sin(meas_y(3))+cos(meas_y(5))*cos(meas_y(3))*sin(meas_y(4)))*esti_x(14),
            (sin(meas_y(5))*cos(meas_y(4)))*esti_x(12) + (cos(meas_y(5))*cos(meas_y(3))+sin(meas_y(3))*sin(meas_y(4))*sin(meas_y(5)))*esti_x(13) + (-cos(meas_y(5))*sin(meas_y(3))+sin(meas_y(4))*sin(meas_y(5))*cos(meas_y(3)))*esti_x(14),
            (-sin(meas_y(4)))*esti_x(12) + (cos(meas_y(4))*sin(meas_y(3)))*esti_x(13) + (cos(meas_y(4))*cos(meas_y(3)))*esti_x(14),
            esti_x(15) + (sin(meas_y(5))*sin(meas_y(4))/cos(meas_y(4)))*esti_x(16) + cos(meas_y(3))*sin(meas_y(4))/cos(meas_y(4))*esti_x(17),
            (cos(meas_y(3)))*esti_x(16) + (sin(meas_y(3)))*esti_x(17),
            (sin(meas_y(3))/cos(meas_y(4)))*esti_x(16) + (cos(meas_y(3))/cos(meas_y(4)))*esti_x(17);
    
    // publish estimate pose
    tf2::Quaternion quat;
    quat.setRPY(esti_x(3), esti_x(4), esti_x(5));
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat, quat_msg);
    esti_pose.pose.pose.position.x = esti_x(0);
    esti_pose.pose.pose.position.y = esti_x(1);
    esti_pose.pose.pose.position.z = esti_x(2);
    esti_pose.pose.pose.orientation.x = quat_msg.x;
    esti_pose.pose.pose.orientation.y = quat_msg.y;
    esti_pose.pose.pose.orientation.z = quat_msg.z;
    esti_pose.pose.pose.orientation.w = quat_msg.w;
    esti_pose.twist.twist.linear.x = esti_x(6);
    esti_pose.twist.twist.linear.y = esti_x(7);
    esti_pose.twist.twist.linear.z = esti_x(8);
    esti_pose.twist.twist.angular.x = esti_x(9);
    esti_pose.twist.twist.angular.y = esti_x(10);
    esti_pose.twist.twist.angular.z = esti_x(11);
    esti_pose.header.stamp = ros::Time::now();
    esti_pose.header.frame_id = "odom_frame";
    esti_pose.child_frame_id = "base_link";
    esti_pose_pub.publish(esti_pose);

    // publish estimate disturbance
    esti_disturbance.pose.pose.position.x = wf_disturbance(0);
    esti_disturbance.pose.pose.position.y = wf_disturbance(1);
    esti_disturbance.pose.pose.position.z = wf_disturbance(2);
    esti_disturbance.twist.twist.angular.x = wf_disturbance(3);
    esti_disturbance.twist.twist.angular.y = wf_disturbance(4);
    esti_disturbance.twist.twist.angular.z = wf_disturbance(5);
    esti_disturbance.header.stamp = ros::Time::now();
    esti_disturbance.header.frame_id = "odom_frame";
    esti_disturbance.child_frame_id = "base_link";
    esti_disturbance_pub.publish(esti_disturbance);

    // publish applied disturbance
    applied_disturbance.pose.pose.position.x = applied_wrench.fx;
    applied_disturbance.pose.pose.position.y = applied_wrench.fy;
    applied_disturbance.pose.pose.position.z = applied_wrench.fz;
    applied_disturbance.twist.twist.angular.x = applied_wrench.tx;
    applied_disturbance.twist.twist.angular.y = applied_wrench.ty;
    applied_disturbance.twist.twist.angular.z = applied_wrench.tz;
    applied_disturbance.header.stamp = ros::Time::now();
    applied_disturbance.header.frame_id = "odom_frame";
    applied_disturbance.child_frame_id = "base_link";
    applied_disturbance_pub.publish(applied_disturbance);

    // print estimate disturbance
    if(cout_counter > 2){
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        std::cout << "tau_X:  " << meas_y(12) << "  tau_Y:  " << meas_y(13) << "  tau_Z:  " << meas_y(14) << "  tau_K:  " << meas_y(15) << "  tau_M:  " << meas_y(16) << "  tau_N:  " << meas_y(17) << std::endl;
        // std::cout << "acc_x:  " << body_acc.x << "  acc_y:  " << body_acc.y << "  acc_z:  " << body_acc.z << std::endl;
        // std::cout << "acc_phi:  " << body_acc.phi << "  acc_theta:  " << body_acc.theta << "  acc_psi:  " << body_acc.psi << std::endl;
        std::cout << "ref_x:    " << acados_in.yref[0][0] << "\tref_y:   " << acados_in.yref[0][1] << "\tref_z:    " << acados_in.yref[0][2] << "\tref_yaw:    " << yaw_ref << std::endl;
        std::cout << "pos_x: " << meas_y(0) << "  pos_y: " << meas_y(1) << "  pos_z: " << meas_y(2) << " phi: " << meas_y(3) << "  theta: " << meas_y(4) << "  psi: " << meas_y(5) <<std::endl;
        // std::cout << "esti_x: " << esti_x(0) << "  esti_y: " << esti_x(1) << "  esti_z: " << esti_x(2) << " esti_phi: " << esti_x(3) << "  esti_theta: " << esti_x(4) << "  esti_psi: " << esti_x(5) <<std::endl;
        std::cout << "error_x:  " << error_pose.pose.pose.position.x << "  error_y:  " << error_pose.pose.pose.position.y << "  error_z:  " << error_pose.pose.pose.position.z << std::endl;
        std::cout << "applied force x:  " << applied_wrench.fx << "\tforce y:  " << applied_wrench.fy << "\tforce_z:  " << applied_wrench.fz << "\ttorque_z:  " << applied_wrench.tz << std::endl;
        // std::cout << "(body frame) disturbance X: " << esti_x(12) << "    disturbance Y: " << esti_x(13) << "    disturbance Z: " << esti_x(14) << "    disturbance N: " << esti_x(17) << std::endl;
        std::cout << "(world frame) disturbance x: " << wf_disturbance(0) << "    disturbance y: " << wf_disturbance(1) << "    disturbance z: " << wf_disturbance(2) << std::endl;
        std::cout << "(world frame) disturbance phi: " << wf_disturbance(3) << "    disturbance theta: " << wf_disturbance(4) << "    disturbance psi: " << wf_disturbance(5) << std::endl;
        std::cout << "estimated added mass Xu':  " << theta_X(0) << "  Yv':  " << theta_Y(0) << "  Zw':  " << theta_Z(0) << "  Nr':  " << theta_N(0) << std::endl;
        std::cout << "estimated D_L Xu:  " << theta_X(1) << "  Yv:  " << theta_Y(1) << "  Zw:  " << theta_Z(1) << "  Nr:  " << theta_N(1) << std::endl;  
        std::cout << "estimated D_NL Xu|u|:  " << theta_X(3) << "  Yv|v|:  " << theta_Y(3) << "  Zw|w|:  " << theta_Z(3) << "  Nr|r|:  " << theta_N(3) << std::endl;  
        std::cout << "estimated ext disturbance Xext:  " << wf_env(0) << "  Yext:  " << wf_env(1) << "  Zext:  " << wf_env(2) << "  Next:  " << wf_env(5) << std::endl;
        std::cout << "estimated ext disturbance Xext (body):  " << theta_X(2) << "  Yext:  " << theta_Y(2) << "  Zext:  " << theta_Z(2) << "  Next:  " << theta_N(2) << std::endl;
        std::cout << "F-test X:  " << RLSX_F << "  F-test Y:  " << RLSY_F << "  F-test Z:  " << RLSZ_F << "  F-test N:  " << RLSN_F << std::endl;
        std::cout << "VFF lambda_X:  " << lambda_X << "  lambda_Y:  " << lambda_Y << "  lambda_Z:  " << lambda_Z << "  lambda_N:  " << lambda_N <<std::endl; 
        std::cout << "solve_time: "<< acados_out.cpu_time << "\tkkt_res: " << acados_out.kkt_res << "\tacados_status: " << acados_out.status << std::endl;
        std::cout << "ros_time:   " << std::fixed << ros::Time::now().toSec() << std::endl;
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        cout_counter = 0;
    }
    else{
        cout_counter++;
    }
}

// 4th order RK for integration
MatrixXd BLUEROV2_AMPC::RK4(MatrixXd x, MatrixXd u)
{
    Matrix<double,18,1> k1;
    Matrix<double,18,1> k2;
    Matrix<double,18,1> k3;
    Matrix<double,18,1> k4;

    k1 = f(x, u) * dt;
    k2 = f(x+k1/2, u) * dt;
    k3 = f(x+k2/3, u) * dt;
    k4 = f(x+k3, u) * dt;

    return x + (k1+2*k2+2*k3+k4)/6;
}

// Define system dynamics function
MatrixXd BLUEROV2_AMPC::f(MatrixXd x, MatrixXd u)
{
    // Define system dynamics
    Matrix<double,18,1> xdot;

    KAu = K*u;
    xdot << (cos(x(5))*cos(x(4)))*x(6) + (-sin(x(5))*cos(x(3))+cos(x(5))*sin(x(4))*sin(x(3)))*x(7) + (sin(x(5))*sin(x(3))+cos(x(5))*cos(x(3))*sin(x(4)))*x(8),  //xdot
            (sin(x(5))*cos(x(4)))*x(6) + (cos(x(5))*cos(x(3))+sin(x(3))*sin(x(4))*sin(x(5)))*x(7) + (-cos(x(5))*sin(x(3))+sin(x(4))*sin(x(5))*cos(x(3)))*x(8),
            (-sin(x(4)))*x(6) + (cos(x(4))*sin(x(3)))*x(7) + (cos(x(4))*cos(x(3)))*x(8),
            x(9) + (sin(x(5))*sin(x(4))/cos(x(4)))*x(10) + cos(x(3))*sin(x(4))/cos(x(4))*x(11),
            (cos(x(3)))*x(10) + (sin(x(3)))*x(11),
            (sin(x(3))/cos(x(4)))*x(10) + (cos(x(3))/cos(x(4)))*x(11), 
            invM(0,0)*(KAu(0)+mass*x(11)*x(7)-mass*x(10)*x(8)-bouyancy*sin(x(4))+x(12)+Dl(0,0)*x(6)),    // xddot: M^-1[tau+w-C-g-D]
            invM(1,1)*(KAu(1)-mass*x(11)*x(6)+mass*x(9)*x(8)+bouyancy*cos(x(4))*sin(x(3))+x(13)+Dl(1,1)*x(7)),
            invM(2,2)*(KAu(2)+mass*x(10)*x(6)-mass*x(9)*x(7)+bouyancy*cos(x(4))*cos(x(3))+x(14)+Dl(2,2)*x(8)),
            invM(3,3)*(KAu(3)+(Iy-Iz)*x(10)*x(11)-mass*ZG*g*cos(x(4))*sin(x(3))+x(15)+Dl(3,3)*x(9)),
            invM(4,4)*(KAu(4)+(Iz-Ix)*x(9)*x(11)-mass*ZG*g*sin(x(4))+x(16)+Dl(4,4)*x(10)),
            invM(5,5)*(KAu(5)-(Iy-Ix)*x(9)*x(10)+x(17)+Dl(5,5)*x(11)),
            0,0,0,0,0,0;
            
    return xdot;
}

// Define measurement model function (Z = Hx, Z: measurement vector [x,xdot,tau]; X: state vector [x,xdot,disturbance])
MatrixXd BLUEROV2_AMPC::h(MatrixXd x)
{
    // Define measurement model
    Matrix<double,18,1> y;
    y << x(0),x(1),x(2),x(3),x(4),x(5),
        x(6),x(7),x(8),x(9),x(10),x(11),
        M(0,0)*body_acc.x-mass*x(11)*x(7)+mass*x(10)*x(8)+bouyancy*sin(x(4))-x(12)-Dl(0,0)*x(6),        
        M(1,1)*body_acc.y+mass*x(11)*x(6)-mass*x(9)*x(8)-bouyancy*cos(x(4))*sin(x(3))-x(13)-Dl(1,1)*x(7),
        M(2,2)*body_acc.z-mass*x(10)*x(6)+mass*x(9)*x(7)-bouyancy*cos(x(4))*cos(x(3))-x(14)-Dl(2,2)*x(8),
        M(3,3)*body_acc.phi-(Iy-Iz)*x(10)*x(11)+mass*ZG*g*cos(x(4))*sin(x(3))-x(15)-Dl(3,3)*x(9),
        M(4,4)*body_acc.theta-(Iz-Ix)*x(9)*x(11)+mass*ZG*g*sin(x(4))-x(16)-Dl(4,4)*x(10),
        M(5,5)*body_acc.psi+(Iy-Ix)*x(9)*x(10)-x(17)-Dl(5,5)*x(11);

    return y;
}

// Define function to compute Jacobian of system dynamics at current state and input
MatrixXd BLUEROV2_AMPC::compute_jacobian_F(MatrixXd x, MatrixXd u)
{
    // Define Jacobian of system dynamics
    Matrix<double,18,18> F;
    double d = 1e-6;                    // finite difference step size
    VectorXd f0 = RK4(x, u);
    for (int i = 0; i < n; i++){
        VectorXd x1 = x;
        x1(i) += d;
        VectorXd f1 = RK4(x1, u);
        F.col(i) = (f1-f0)/d;
    }
    return F;
}

// Define function to compute Jacobian of measurement model at predicted state
MatrixXd BLUEROV2_AMPC::compute_jacobian_H(MatrixXd x)
{
    // Define Jacobian of measurement model
    Matrix<double,18,18> H;
    double d = 1e-6;                    // finite difference step size
    VectorXd f0 = h(x);
    for (int i = 0; i < n; i++){
        VectorXd x1 = x;
        x1(i) += d;
        VectorXd f1 = h(x1);
        H.col(i) = (f1-f0)/d;
    }
    return H;
}

// Update procedure of the RLS-FF
void BLUEROV2_AMPC::RLSFF()
{
    double e;
    MatrixXd K;
    double threshold = 0.8;

    // direction X------------------------------------------------------------------------------------
    VectorXd RLSX_x(numParams);
    double RLSX_y = esti_x(12);
    RLSX_x << body_acc.x, v_linear_body[0], 1, v_linear_body[0]*abs(v_linear_body[0]);

    e = RLSX_y - RLSX_x.dot(theta_X); // Compute the prediction error

    // store error
    Xerror_n.push_back(e);               
    Xerror_d.push_back(e);
    if (Xerror_n.size() > FF_n) {
        Xerror_n.erase(Xerror_n.begin());
    }
    if (Xerror_d.size() > FF_d) {
        Xerror_d.erase(Xerror_d.begin());
    }

    // Calculate the F-statistic
    double error_n_mean = std::accumulate(Xerror_n.begin(), Xerror_n.end(), 0.0) / Xerror_n.size();
    double error_n_variance = 0.0;
    for (const auto& value : Xerror_n) {     // calculate error variance of short window
        error_n_variance += std::pow(value - error_n_mean, 2);
    }
    error_n_variance /= Xerror_n.size();

    double error_d_mean = std::accumulate(Xerror_d.begin(), Xerror_d.end(), 0.0) / Xerror_d.size();
    double error_d_variance = 0.0;
    for (const auto& value : Xerror_d) {     // calculate error variance of long window
        error_d_variance += std::pow(value - error_d_mean, 2);
    }
    error_d_variance /= Xerror_d.size();

    RLSX_F = error_n_variance/error_d_variance;     // calculate F-test

    // Update the forgetting factor
    if (RLSX_F > threshold) {
        if (lambda_X - 0.01 >= 0.5)
        {
            lambda_X -= 0.01;
        }
        else{
            lambda_X = 0.5;
        }
        
    } else {
        if (lambda_X + 0.01 <= 1)
        {
            lambda_X += 0.01;
        }
        else
        {
            lambda_X = 1;
        }
    }

    K = RLSX_P * RLSX_x / (lambda_X + RLSX_x.dot(RLSX_P * RLSX_x)); // Compute the Kalman gain
    theta_X += K * e; // Update the parameter vector
    RLSX_P = (RLSX_P - K * RLSX_x.transpose() * RLSX_P) / lambda_X; // Update the covariance matrix

    // direction Y------------------------------------------------------------------------------------
    VectorXd RLSY_x(numParams);
    double RLSY_y = esti_x(13);
    RLSY_x << body_acc.y, v_linear_body[1], 1, v_linear_body[1]*abs(v_linear_body[1]);

    e = RLSY_y - RLSY_x.dot(theta_Y); // Compute the prediction error

    // store error
    Yerror_n.push_back(e);               
    Yerror_d.push_back(e);
    if (Yerror_n.size() > FF_n) {
        Yerror_n.erase(Yerror_n.begin());
    }
    if (Yerror_d.size() > FF_d) {
        Yerror_d.erase(Yerror_d.begin());
    }

    // Calculate the F-statistic
    error_n_variance = 0.0;
    error_n_mean = std::accumulate(Yerror_n.begin(), Yerror_n.end(), 0.0) / Yerror_n.size();
    for (const auto& value : Yerror_n) {     // calculate error variance of short window
        error_n_variance += std::pow(value - error_n_mean, 2);
    }
    error_n_variance /= Yerror_n.size();

    error_d_variance = 0.0;
    error_d_mean = std::accumulate(Yerror_d.begin(), Yerror_d.end(), 0.0) / Yerror_d.size();
    for (const auto& value : Yerror_d) {     // calculate error variance of long window
        error_d_variance += std::pow(value - error_d_mean, 2);
    }
    error_d_variance /= Yerror_d.size();

    RLSY_F = error_n_variance/error_d_variance;     // calculate F-test

    // Update the forgetting factor
    if (RLSY_F > threshold) {
        if (lambda_Y - 0.01 >= 0.5)
        {
            lambda_Y -= 0.01;
        }
        else{
            lambda_Y = 0.5;
        }
        
    } else {
        if (lambda_Y + 0.01 <= 1)
        {
            lambda_Y += 0.01;
        }
        else
        {
            lambda_Y = 1;
        }
    }

    K = RLSY_P * RLSY_x / (lambda_Y + RLSY_x.dot(RLSY_P * RLSY_x)); // Compute the Kalman gain
    theta_Y += K * e; // Update the parameter vector
    RLSY_P = (RLSY_P - K * RLSY_x.transpose() * RLSY_P) / lambda_Y; // Update the covariance matrix

    // direction Z------------------------------------------------------------------------------------
    VectorXd RLSZ_x(numParams);
    double RLSZ_y = esti_x(14);
    RLSZ_x << body_acc.z, v_linear_body[2], 1, v_linear_body[2]*abs(v_linear_body[2]);

    e = RLSZ_y - RLSZ_x.dot(theta_Z); // Compute the prediction error

    // store error
    Zerror_n.push_back(e);               
    Zerror_d.push_back(e);
    if (Zerror_n.size() > FF_n) {
        Zerror_n.erase(Zerror_n.begin());
    }
    if (Zerror_d.size() > FF_d) {
        Zerror_d.erase(Zerror_d.begin());
    }

    // Calculate the F-statistic
    error_n_variance = 0.0;
    error_n_mean = std::accumulate(Zerror_n.begin(), Zerror_n.end(), 0.0) / Zerror_n.size();
    for (const auto& value : Zerror_n) {     // calculate error variance of short window
        error_n_variance += std::pow(value - error_n_mean, 2);
    }
    error_n_variance /= Zerror_n.size();

    error_d_variance = 0.0;
    error_d_mean = std::accumulate(Zerror_d.begin(), Zerror_d.end(), 0.0) / Zerror_d.size();
    for (const auto& value : Zerror_d) {     // calculate error variance of long window
        error_d_variance += std::pow(value - error_d_mean, 2);
    }
    error_d_variance /= Zerror_d.size();

    RLSZ_F = error_n_variance/error_d_variance;     // calculate F-test

    // Update the forgetting factor
    if (RLSZ_F > threshold) {
        if (lambda_Z - 0.01 >= 0.5)
        {
            lambda_Z -= 0.01;
        }
        else{
            lambda_Z = 0.5;
        }
        
    } else {
        if (lambda_Z + 0.01 <= 1)
        {
            lambda_Z += 0.01;
        }
        else
        {
            lambda_Z = 1;
        }
    }

    K = RLSZ_P * RLSZ_x / (lambda_Z + RLSZ_x.dot(RLSZ_P * RLSZ_x)); // Compute the Kalman gain
    theta_Z += K * e; // Update the parameter vector
    RLSZ_P = (RLSZ_P - K * RLSZ_x.transpose() * RLSZ_P) / lambda_Z; // Update the covariance matrix

    // direction N------------------------------------------------------------------------------------
    VectorXd RLSN_x(numParams);
    double RLSN_y = esti_x(17);
    RLSN_x << body_acc.psi, v_angular_body[2], 1, v_angular_body[2]*abs(v_angular_body[2]);

    e = RLSN_y - RLSN_x.dot(theta_N); // Compute the prediction error

    // store error
    Nerror_n.push_back(e);               
    Nerror_d.push_back(e);
    if (Nerror_n.size() > FF_n) {
        Nerror_n.erase(Nerror_n.begin());
    }
    if (Nerror_d.size() > FF_d) {
        Nerror_d.erase(Nerror_d.begin());
    }

    // Calculate the F-statistic
    error_n_variance = 0.0;
    error_n_mean = std::accumulate(Nerror_n.begin(), Nerror_n.end(), 0.0) / Nerror_n.size();
    for (const auto& value : Nerror_n) {     // calculate error variance of short window
        error_n_variance += std::pow(value - error_n_mean, 2);
    }
    error_n_variance /= Nerror_n.size();

    error_d_variance = 0.0;
    error_d_mean = std::accumulate(Nerror_d.begin(), Nerror_d.end(), 0.0) / Nerror_d.size();
    for (const auto& value : Nerror_d) {     // calculate error variance of long window
        error_d_variance += std::pow(value - error_d_mean, 2);
    }
    error_d_variance /= Nerror_d.size();

    RLSN_F = error_n_variance/error_d_variance;     // calculate F-test

    // Update the forgetting factor
    if (RLSN_F > threshold) {
        if (lambda_N - 0.01 >= 0.5)
        {
            lambda_N -= 0.01;
        }
        else{
            lambda_N = 0.5;
        }
        
    } else {
        if (lambda_N + 0.01 <= 1)
        {
            lambda_N += 0.01;
        }
        else
        {
            lambda_N = 1;
        }
    }

    K = RLSN_P * RLSN_x / (lambda_N + RLSN_x.dot(RLSN_P * RLSN_x)); // Compute the Kalman gain
    theta_N += K * e; // Update the parameter vector
    RLSN_P = (RLSN_P - K * RLSN_x.transpose() * RLSN_P) / lambda_N; // Update the covariance matrix

    // publish estimated added mass
    esti_added_mass.pose.pose.position.x = theta_X(0);
    esti_added_mass.pose.pose.position.y = theta_Y(0);
    esti_added_mass.pose.pose.position.z = theta_Z(0);
    esti_added_mass.twist.twist.angular.x = 0;
    esti_added_mass.twist.twist.angular.y = 0;
    esti_added_mass.twist.twist.angular.z = theta_N(0);
    esti_added_mass.header.stamp = ros::Time::now();
    esti_added_mass.header.frame_id = "odom_frame";
    esti_added_mass.child_frame_id = "base_link";
    esti_added_mass_pub.publish(esti_added_mass);

    // publish estimated damping
    esti_damping.pose.pose.position.x = theta_X(1);
    esti_damping.pose.pose.position.y = theta_Y(1);
    esti_damping.pose.pose.position.z = theta_Z(1);
    esti_damping.twist.twist.angular.x = 0;
    esti_damping.twist.twist.angular.y = 0;
    esti_damping.twist.twist.angular.z = theta_N(1);
    esti_damping.header.stamp = ros::Time::now();
    esti_damping.header.frame_id = "odom_frame";
    esti_damping.child_frame_id = "base_link";
    esti_damping_pub.publish(esti_damping);

    // body frame disturbance to inertial frame
    wf_env << (cos(local_euler.psi)*cos(local_euler.theta))*theta_X(2) + (-sin(local_euler.psi)*cos(local_euler.phi)+cos(local_euler.psi)*sin(local_euler.theta)*sin(local_euler.phi))*theta_Y(2) + (sin(local_euler.psi)*sin(local_euler.phi)+cos(local_euler.psi)*cos(local_euler.phi)*sin(local_euler.theta))*theta_Z(2),
            (sin(local_euler.psi)*cos(local_euler.theta))*theta_X(2) + (cos(local_euler.psi)*cos(local_euler.phi)+sin(local_euler.phi)*sin(local_euler.theta)*sin(local_euler.psi))*theta_Y(2) + (-cos(local_euler.psi)*sin(local_euler.phi)+sin(local_euler.theta)*sin(local_euler.psi)*cos(local_euler.phi))*theta_Z(2),
            (-sin(local_euler.theta))*theta_X(2) + (cos(local_euler.theta)*sin(local_euler.phi))*theta_Y(2) + (cos(local_euler.theta)*cos(local_euler.phi))*theta_Z(2),
            cos(local_euler.phi)*sin(local_euler.theta)/cos(local_euler.theta)*theta_N(2),
            (sin(local_euler.phi))*theta_N(2),
            (cos(local_euler.phi)/cos(local_euler.theta))*theta_N(2);

    // // body frame disturbance to inertial frame
    // wf_disturbance << (cos(meas_y(5))*cos(meas_y(4)))*esti_x(12) + (-sin(meas_y(5))*cos(meas_y(3))+cos(meas_y(5))*sin(meas_y(4))*sin(meas_y(3)))*esti_x(13) + (sin(meas_y(5))*sin(meas_y(3))+cos(meas_y(5))*cos(meas_y(3))*sin(meas_y(4)))*esti_x(14),
    //         (sin(meas_y(5))*cos(meas_y(4)))*esti_x(12) + (cos(meas_y(5))*cos(meas_y(3))+sin(meas_y(3))*sin(meas_y(4))*sin(meas_y(5)))*esti_x(13) + (-cos(meas_y(5))*sin(meas_y(3))+sin(meas_y(4))*sin(meas_y(5))*cos(meas_y(3)))*esti_x(14),
    //         (-sin(meas_y(4)))*esti_x(12) + (cos(meas_y(4))*sin(meas_y(3)))*esti_x(13) + (cos(meas_y(4))*cos(meas_y(3)))*esti_x(14),
    //         esti_x(15) + (sin(meas_y(5))*sin(meas_y(4))/cos(meas_y(4)))*esti_x(16) + cos(meas_y(3))*sin(meas_y(4))/cos(meas_y(4))*esti_x(17),
    //         (cos(meas_y(3)))*esti_x(16) + (sin(meas_y(3)))*esti_x(17),
    //         (sin(meas_y(3))/cos(meas_y(4)))*esti_x(16) + (cos(meas_y(3))/cos(meas_y(4)))*esti_x(17);

    // publish estimated environmental disturbance
    esti_env.pose.pose.position.x = theta_X(2);
    esti_env.pose.pose.position.y = theta_Y(2);
    esti_env.pose.pose.position.z = theta_Z(2);
    esti_env.twist.twist.angular.x = 0;
    esti_env.twist.twist.angular.y = 0;
    esti_env.twist.twist.angular.z = theta_N(2);
    esti_env.header.stamp = ros::Time::now();
    esti_env.header.frame_id = "odom_frame";
    esti_env.child_frame_id = "base_link";
    esti_env_pub.publish(esti_env);
    // esti_env.pose.pose.position.x = wf_env(0);
    // esti_env.pose.pose.position.y = wf_env(1);
    // esti_env.pose.pose.position.z = wf_env(2);
    // esti_env.twist.twist.angular.x = 0;
    // esti_env.twist.twist.angular.y = 0;
    // esti_env.twist.twist.angular.z = wf_env(5);
    // esti_env.header.stamp = ros::Time::now();
    // esti_env.header.frame_id = "odom_frame";
    // esti_env.child_frame_id = "base_link";
    // esti_env_pub.publish(esti_env);

    // publish estimated nonlinear damping
    esti_Ndamping.pose.pose.position.x = theta_X(3);
    esti_Ndamping.pose.pose.position.y = theta_Y(3);
    esti_Ndamping.pose.pose.position.z = theta_Z(3);
    esti_Ndamping.twist.twist.angular.x = 0;
    esti_Ndamping.twist.twist.angular.y = 0;
    esti_Ndamping.twist.twist.angular.z = theta_N(3);
    esti_Ndamping.header.stamp = ros::Time::now();
    esti_Ndamping.header.frame_id = "odom_frame";
    esti_Ndamping.child_frame_id = "base_link";
    esti_Ndamping_pub.publish(esti_Ndamping);
}


// Apply body wrench at the center of the vehicle
void BLUEROV2_AMPC::applyBodyWrench()
{
    // initialize random value
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distribution(0.5, 1.0);

    // initialize disturbance data from file
    std::vector<double> data_fx;
    std::vector<double> data_fy;
    std::vector<double> data_fz;
    std::vector<double> data_tz;
    const char * fx_c = WRENCH_FX.c_str();
    const char * fy_c = WRENCH_FY.c_str();
    const char * fz_c = WRENCH_FZ.c_str();
    const char * tz_c = WRENCH_TZ.c_str();

    if(READ_WRENCH == 0){
        // generate periodical disturbance
        if(dis_time > periodic_counter*M_PI)
        {
            amplitudeScalingFactor_X = distribution(gen)*5;
            amplitudeScalingFactor_Y = distribution(gen)*5;
            amplitudeScalingFactor_Z = distribution(gen)*5;
            amplitudeScalingFactor_N = distribution(gen)*5;
            periodic_counter++;
        }
        applied_wrench.fx = sin(dis_time)*amplitudeScalingFactor_X;
        applied_wrench.fy = sin(dis_time)*amplitudeScalingFactor_Y;
        applied_wrench.fz = sin(dis_time)*amplitudeScalingFactor_Z;
        applied_wrench.tz = 0;
        dis_time = dis_time+dt*0.5;     // frequency of the disturbance
    }
    else if(READ_WRENCH == 1){
        // generate constant disturbance
        applied_wrench.fx = 10;
        applied_wrench.fy = 10;
        applied_wrench.fz = 10;
        applied_wrench.tz = 0;
    }
    else if(READ_WRENCH == 2){
        // read disturbance from file
        // read force x
        std::ifstream fx_file(fx_c);
        if (!fx_file.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return;
        }
        double value_fx;
        while (fx_file >> value_fx) {
            data_fx.push_back(value_fx); // Load the data into the vector
        }
        fx_file.close();

        // read force y
        std::ifstream fy_file(fy_c);
        if (!fy_file.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return;
        }
        double value_fy;
        while (fy_file >> value_fy) {
            data_fy.push_back(value_fy); // Load the data into the vector
        }
        fy_file.close();

        // read force z
        std::ifstream fz_file(fz_c);
        if (!fz_file.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return;
        }
        double value_fz;
        while (fz_file >> value_fz) {
            data_fz.push_back(value_fz); // Load the data into the vector
        }
        fz_file.close();

        // read torque z
        std::ifstream tz_file(tz_c);
        if (!tz_file.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return;
        }
        double value_tz;
        while (tz_file >> value_tz) {
            data_tz.push_back(value_tz); // Load the data into the vector
        }
        tz_file.close();

        applied_wrench.fx  = data_fx[fx_counter];
        applied_wrench.fy  = data_fy[fx_counter];
        applied_wrench.fz  = data_fz[fx_counter];
        applied_wrench.tz  = data_tz[fx_counter];
        fx_counter++;
    }
    
    // call ros service apply_body_wrench
    body_wrench.request.body_name = "bluerov2/base_link";
    body_wrench.request.start_time = ros::Time(0.0);
    body_wrench.request.reference_frame = "world";
    body_wrench.request.duration = ros::Duration(1000);
    body_wrench.request.reference_point.x = 0.0;
    body_wrench.request.reference_point.y = 0.0;
    body_wrench.request.reference_point.z = 0.0;
    body_wrench.request.wrench.force.x = applied_wrench.fx;
    body_wrench.request.wrench.force.y = applied_wrench.fy;
    body_wrench.request.wrench.force.z = applied_wrench.fz;
    body_wrench.request.wrench.torque.x = applied_wrench.tx;
    body_wrench.request.wrench.torque.y = applied_wrench.ty;
    body_wrench.request.wrench.torque.z = applied_wrench.tz;
    client.call(body_wrench);
    
}
