#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

#include <iostream>
#include <fstream>
#include <cmath>

#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include "bluerov2_model/bluerov2_model.h"
#include "acados_solver_bluerov2.h"
bool start_sub;
class NMPC
    {
        private:
        float yaw_sum = 0;      // yaw degree as continous number
        float pre_yaw = 0;      // former state yaw degree
        float yaw_diff;         // yaw degree difference in every step
        float yaw_ref;          // yaw degree reference in form of (-pi, pi)
        float yaw_error;        // yaw degree error
        
        enum SystemStates{
            x = 0,
            y = 1,
            z = 2,
            phi = 3,
            theta = 4,
            psi = 5,
            u = 6,
            v = 7,
            w = 8,
            p = 9,
            q = 10,
            r = 11,
        };

        enum ControlInputs{
            u1 = 0,
            u2 = 1,
            u3 = 2,
            u4 = 3,
        };

        struct SolverInput{
            double x0[BLUEROV2_NX];
            double yref[BLUEROV2_N+1][BLUEROV2_NY];
        };

        struct SolverOutput{
            double u0[BLUEROV2_NU];
            double x1[BLUEROV2_NX];
            double status, kkt_res, cpu_time;
        };

        struct Euler{
            double phi;
            double theta;
            double psi;
        };

        // ROS subscriber and publisher
        ros::Subscriber pose_gt_sub;

        ros::Publisher thrust0_pub;
        ros::Publisher thrust1_pub;
        ros::Publisher thrust2_pub;
        ros::Publisher thrust3_pub;
        ros::Publisher thrust4_pub;
        ros::Publisher thrust5_pub;

        //ros::Publisher marker_pub;

        ros::Publisher ref_pose_pub;

        ros::Publisher error_pose_pub;

        ros::Publisher control_input0_pub;
        ros::Publisher control_input1_pub;
        ros::Publisher control_input2_pub;
        ros::Publisher control_input3_pub;

        // ROS message variables
        nav_msgs::Odometry pose_gt;
        Euler local_euler;

        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust0;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust1;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust2;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust3;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust4;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust5;
        
        //visualization_msgs::Marker marker;

        nav_msgs::Odometry ref_pose;

        nav_msgs::Odometry error_pose;

        uuv_gazebo_ros_plugins_msgs::FloatStamped control_input0;
        uuv_gazebo_ros_plugins_msgs::FloatStamped control_input1;
        uuv_gazebo_ros_plugins_msgs::FloatStamped control_input2;
        uuv_gazebo_ros_plugins_msgs::FloatStamped control_input3;

        // Acados variables
        SolverInput acados_in;
        SolverOutput acados_out;
        int acados_status;   
        bluerov2_solver_capsule * mpc_capsule = bluerov2_acados_create_capsule();
        
        // Trajectory variables
        std::vector<std::vector<double>> trajectory;
        int line_number = 0;
        int number_of_steps = 0;

        // Other variables
        tf::Quaternion tf_quaternion;
        int cout_counter = 0;
        double logger_time;

        public:

        NMPC(ros::NodeHandle& nh, const std::string& ref_traj)
        {

            // Pre-load the trajectory
            const char * c = ref_traj.c_str();
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

            // ROS Subscriber & Publisher
            pose_gt_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/pose_gt", 20, &NMPC::pose_gt_cb, this);
            thrust0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/0/input",20);
            thrust1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/1/input",20);
            thrust2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/2/input",20);
            thrust3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/3/input",20);
            thrust4_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/4/input",20);
            thrust5_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/5/input",20);
            //marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 20);
            ref_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/mpc/reference",20);
            error_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/mpc/error",20);
            control_input0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/0",20);
            control_input1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/1",20);
            control_input2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/2",20);
            control_input3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/3",20);
            // Initialize
            for(unsigned int i=0; i < BLUEROV2_NU; i++) acados_out.u0[i] = 0.0;
            for(unsigned int i=0; i < BLUEROV2_NX; i++) acados_in.x0[i] = 0.0;

        }
        void pose_gt_cb(const nav_msgs::Odometry::ConstPtr& pose)
        {
            start_sub = true;
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
       
        int readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data)
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

        void ref_cb(int line_to_read)
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

        void run()
        {
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
            
            acados_in.x0[x] = pose_gt.pose.pose.position.x;
            acados_in.x0[y] = pose_gt.pose.pose.position.y;
            acados_in.x0[z] = pose_gt.pose.pose.position.z;
            acados_in.x0[phi] = local_euler.phi;
            acados_in.x0[theta] = local_euler.theta;
            acados_in.x0[psi] = yaw_sum;
            acados_in.x0[u] = pose_gt.twist.twist.linear.x;
            acados_in.x0[v] = pose_gt.twist.twist.linear.y;
            acados_in.x0[w] = pose_gt.twist.twist.linear.z;
            acados_in.x0[p] = pose_gt.twist.twist.angular.x;
            acados_in.x0[q] = pose_gt.twist.twist.angular.y;
            acados_in.x0[r] = pose_gt.twist.twist.angular.z;
            
            // change into form of (-pi, pi)
            if(sin(acados_in.yref[0][5]) >= 0)
            {
                yaw_ref = fmod(acados_in.yref[0][5],M_PI);
            }
            else{
                yaw_ref = -M_PI + fmod(acados_in.yref[0][5],M_PI);
            }
            
            ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "lbx", acados_in.x0);
            ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "ubx", acados_in.x0);
            
            ref_cb(line_number); 
            line_number++;
            
            for (unsigned int i = 0; i <= BLUEROV2_N; i++)
                {            
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

            //rotor constant 0.026546960744430276
            /*
            thrust0.data=80/(1+exp(-4*pow((-acados_out.u0[0]+acados_out.u0[1]+acados_out.u0[3]),3)))-40;
            thrust1.data=80/(1+exp(-4*pow((-acados_out.u0[0]-acados_out.u0[1]-acados_out.u0[3]),3)))-40;
            thrust2.data=80/(1+exp(-4*pow((acados_out.u0[0]+acados_out.u0[1]-acados_out.u0[3]),3)))-40;
            thrust3.data=80/(1+exp(-4*pow((acados_out.u0[0]-acados_out.u0[1]+acados_out.u0[3]),3)))-40;
            thrust4.data=80/(1+exp(-4*pow((-acados_out.u0[2]),3)))-40;
            thrust5.data=80/(1+exp(-4*pow((-acados_out.u0[2]),3)))-40;
            */
            // publish thrusts
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

            /*
            // publish trajectory marker on rviz
            marker.header.frame_id = "map";
            marker.ns = "trajectory";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.1;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            geometry_msgs::Point current_point;
            current_point.x = acados_in.yref[0][0];
            current_point.y = acados_in.yref[0][1];
            current_point.z = acados_in.yref[0][2];
            marker.points.push_back(current_point);
            
            marker_pub.publish(marker);
            */
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

            /*Mission information cout**********************************************/        
            if(cout_counter > 2){ //reduce cout rate
                std::cout << "------------------------------------------------------------------------------------------------" << std::endl;
                std::cout << "x_ref:    " << acados_in.yref[0][0] << "\ty_ref:   " << acados_in.yref[0][1] << "\tz_ref:    " << acados_in.yref[0][2] << "\tyaw_ref:    " << yaw_ref << std::endl;
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
    };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluerov2_mpc");
    ros::NodeHandle nh;
    std::string ref_traj;
    nh.getParam("/bluerov2_mpc_node/ref_traj", ref_traj);
    NMPC nmpc(nh, ref_traj);
    ros::Rate loop_rate(20);
    start_sub = false;
    while(ros::ok()){
        if(start_sub == true){
            nmpc.run();
        }
        //nmpc.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}