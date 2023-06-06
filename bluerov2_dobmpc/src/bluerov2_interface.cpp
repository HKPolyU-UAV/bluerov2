#include <bluerov2_interface.h>

BLUEROV2_INTERFACE::BLUEROV2_INTERFACE(ros::NodeHandle& nh){
    // set yaw control mode
    nh.getParam("/bluerov2_dobmpc_node/auto_yaw",AUTO_YAW);
    nh.getParam("/bluerov2_dobmpc_node/ref_traj", REF_TRAJ);

    // Pre-load the trajectory
    const char * c = REF_TRAJ.c_str();
	number_of_steps = readDataFromFile(c, trajectory);
	if (number_of_steps == 0){
		ROS_WARN("Cannot load CasADi optimal trajectory!");
	}
	else{
		ROS_INFO_STREAM("Number of steps of selected trajectory: " << number_of_steps << std::endl);
	}



    // ros subsriber & publisher
    pose_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/pose_gt", 20, &BLUEROV2_INTERFACE::pose_cb, this);
    thrust0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/0/input",20);
    thrust1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/1/input",20);
    thrust2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/2/input",20);
    thrust3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/3/input",20);
    thrust4_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/4/input",20);
    thrust5_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/5/input",20);
    //ref_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/mpc/reference",20);
    //error_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/mpc/error",20);
    //control_input0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/0",20);
    //control_input1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/1",20);
    //control_input2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/2",20);
    //control_input3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/3",20);

}

int BLUEROV2_INTERFACE::readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data)
{
	std::ifstream file(fileName);
	std::string line;
	int number_of_lines = 0;

	if (file.is_open())
	{
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
		return 0;
	}

	return number_of_lines;
}

geometry_msgs::Quaternion BLUEROV2_INTERFACE::rpy2q(const float& euler_phi, const float& euler_theta, const float& euler_psi){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(euler_phi, euler_theta, euler_psi);
    return quaternion;
}

void BLUEROV2_INTERFACE::ref_cb(int line_to_read)
{
    if (BLUEROV2_N+line_to_read+1 <= number_of_steps)  // All ref points within the file
    {
        for (unsigned int i = 0; i <= BLUEROV2_NY; i++)  // Fill all horizon with file data
        {
            mpc_ref.ref_pose[i].position.x = trajectory[i+line_to_read][0];
            mpc_ref.ref_pose[i].position.y = trajectory[i+line_to_read][1];
            mpc_ref.ref_pose[i].position.z = trajectory[i+line_to_read][2];
            ref_quat = rpy2q(trajectory[i+line_to_read][3],trajectory[i+line_to_read][4],trajectory[i+line_to_read][5]);
            mpc_ref.ref_pose[i].orientation.w = ref_quat.w;
            mpc_ref.ref_pose[i].orientation.x = ref_quat.x;
            mpc_ref.ref_pose[i].orientation.y = ref_quat.y;
            mpc_ref.ref_pose[i].orientation.z = ref_quat.z;
            mpc_ref.ref_twist[i].linear.x = trajectory[i+line_to_read][6];
            mpc_ref.ref_twist[i].linear.y = trajectory[i+line_to_read][7];
            mpc_ref.ref_twist[i].linear.z = trajectory[i+line_to_read][8];
            mpc_ref.ref_twist[i].angular.x = trajectory[i+line_to_read][9];
            mpc_ref.ref_twist[i].angular.y = trajectory[i+line_to_read][10];
            mpc_ref.ref_twist[i].angular.z = trajectory[i+line_to_read][11];
            /*
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[i+line_to_read][j];
            }
            */
        }
    }
    else if(line_to_read < number_of_steps)    // Part of ref points within the file
    {
        for (unsigned int i = 0; i < number_of_steps-line_to_read; i++)    // Fill part of horizon with file data
        {
            mpc_ref.ref_pose[i].position.x = trajectory[i+line_to_read][0];
            mpc_ref.ref_pose[i].position.y = trajectory[i+line_to_read][1];
            mpc_ref.ref_pose[i].position.z = trajectory[i+line_to_read][2];
            ref_quat = rpy2q(trajectory[i+line_to_read][3],trajectory[i+line_to_read][4],trajectory[i+line_to_read][5]);
            mpc_ref.ref_pose[i].orientation.w = ref_quat.w;
            mpc_ref.ref_pose[i].orientation.x = ref_quat.x;
            mpc_ref.ref_pose[i].orientation.y = ref_quat.y;
            mpc_ref.ref_pose[i].orientation.z = ref_quat.z;
            mpc_ref.ref_twist[i].linear.x = trajectory[i+line_to_read][6];
            mpc_ref.ref_twist[i].linear.y = trajectory[i+line_to_read][7];
            mpc_ref.ref_twist[i].linear.z = trajectory[i+line_to_read][8];
            mpc_ref.ref_twist[i].angular.x = trajectory[i+line_to_read][9];
            mpc_ref.ref_twist[i].angular.y = trajectory[i+line_to_read][10];
            mpc_ref.ref_twist[i].angular.z = trajectory[i+line_to_read][11];
            /*
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[i+line_to_read][j];
            }
            */
        }

        for (unsigned int i = number_of_steps-line_to_read; i <= BLUEROV2_N; i++)  // Fill the rest horizon with the last point
        {
            mpc_ref.ref_pose[i].position.x = trajectory[number_of_steps-1][0];
            mpc_ref.ref_pose[i].position.y = trajectory[number_of_steps-1][1];
            mpc_ref.ref_pose[i].position.z = trajectory[number_of_steps-1][2];
            ref_quat = rpy2q(trajectory[number_of_steps-1][3],trajectory[number_of_steps-1][4],trajectory[number_of_steps-1][5]);
            mpc_ref.ref_pose[i].orientation.w = ref_quat.w;
            mpc_ref.ref_pose[i].orientation.x = ref_quat.x;
            mpc_ref.ref_pose[i].orientation.y = ref_quat.y;
            mpc_ref.ref_pose[i].orientation.z = ref_quat.z;
            mpc_ref.ref_twist[i].linear.x = trajectory[number_of_steps-1][6];
            mpc_ref.ref_twist[i].linear.y = trajectory[number_of_steps-1][7];
            mpc_ref.ref_twist[i].linear.z = trajectory[number_of_steps-1][8];
            mpc_ref.ref_twist[i].angular.x = trajectory[number_of_steps-1][9];
            mpc_ref.ref_twist[i].angular.y = trajectory[number_of_steps-1][10];
            mpc_ref.ref_twist[i].angular.z = trajectory[number_of_steps-1][11];
            /*
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
            }
            */
        }
    }
    else    // none of ref points within the file
    {
        for (unsigned int i = 0; i <= BLUEROV2_N; i++)  // Fill all horizon with the last point
        {
            mpc_ref.ref_pose[i].position.x = trajectory[number_of_steps-1][0];
            mpc_ref.ref_pose[i].position.y = trajectory[number_of_steps-1][1];
            mpc_ref.ref_pose[i].position.z = trajectory[number_of_steps-1][2];
            ref_quat = rpy2q(trajectory[number_of_steps-1][3],trajectory[number_of_steps-1][4],trajectory[number_of_steps-1][5]);
            mpc_ref.ref_pose[i].orientation.w = ref_quat.w;
            mpc_ref.ref_pose[i].orientation.x = ref_quat.x;
            mpc_ref.ref_pose[i].orientation.y = ref_quat.y;
            mpc_ref.ref_pose[i].orientation.z = ref_quat.z;
            mpc_ref.ref_twist[i].linear.x = trajectory[number_of_steps-1][6];
            mpc_ref.ref_twist[i].linear.y = trajectory[number_of_steps-1][7];
            mpc_ref.ref_twist[i].linear.z = trajectory[number_of_steps-1][8];
            mpc_ref.ref_twist[i].angular.x = trajectory[number_of_steps-1][9];
            mpc_ref.ref_twist[i].angular.y = trajectory[number_of_steps-1][10];
            mpc_ref.ref_twist[i].angular.z = trajectory[number_of_steps-1][11];
            /*
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
            }
            */
        }
    }
}

// get current pose
void BLUEROV2_INTERFACE::pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    pose_gt.header.stamp = msg->header.stamp;
    pose_gt.pose = msg->pose;
    pose_gt.twist = msg->twist;
}

// call acados solver with inputs: current pose, reference, parameter
// publish thrust
void BLUEROV2_INTERFACE::run(){
    ref_cb(line_number);
    line_number++;
    thrusts = controller.solve(pose_gt,mpc_ref,solver_param);
    thrust0 = thrusts[0];
    thrust1 = thrusts[1];
    thrust2 = thrusts[2];
    thrust3 = thrusts[3];
    thrust4 = thrusts[4];
    thrust5 = thrusts[5];

    thrust0_pub.publish(thrust0);
    thrust1_pub.publish(thrust1);
    thrust2_pub.publish(thrust2);
    thrust3_pub.publish(thrust3);
    thrust4_pub.publish(thrust4);
    thrust5_pub.publish(thrust5);
}
