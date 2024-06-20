#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <cmath>
#include <tuple>

using namespace Eigen;

class RandomMotion
{
private:
    // ros::NodeHandle nh;

    // Subscribers & Publishers
    // ros::Subscriber pcl_sub;
    ros::Subscriber pose_gt_sub;
    // ros::Subscriber imu_sub;
    // ros::Subscriber pressure_sub;
    ros::Publisher thrust0_pub;
    ros::Publisher thrust1_pub;
    ros::Publisher thrust2_pub;
    ros::Publisher thrust3_pub;
    ros::Publisher thrust4_pub;
    ros::Publisher thrust5_pub;
    ros::Publisher ref_pose_pub;
    ros::Publisher visual_pose_pub;
    ros::Publisher control_input0_pub;
    ros::Publisher control_input1_pub;
    ros::Publisher control_input2_pub;
    ros::Publisher control_input3_pub;

    // ROS message variables
    // nav_msgs::Odometry pose_gt;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust0;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust2;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust3;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust4;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust5;
    nav_msgs::Odometry ref_pose;
    nav_msgs::Odometry visual_pose;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input0;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input2;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input3;

    // PID parameters
    // double kp = 1;  
    // double ki = 0;  
    // double kd = 0; 
    double prevError;
    double integral;

    // System variables
    struct Euler{
        double phi;
        double theta;
        double psi;
    };
    struct state{
        double x;
        double z;
        double yaw;
    };
    struct controlInputs{
        double u1;
        double u2;
        double u3;
        double u4;
    };
    struct orient{
        double w;
        double x;
        double y;
        double z;
    };
    struct pose{
            double x;
            double y;
            double z;
            double u;
            double v;
            double w;
            double p;
            double q;
            double r;
        };
    Euler local_euler;
    state ref;
    state pos;
    state error;
    controlInputs control_u;
    pose local_pos;
    orient imu_q;
    double dt = 0.05;
    double rotor_constant = 0.026546960744430276;
    double safety_dis = 1.79;
    double buffer = 0.5;
    double sway = -4;
    double initial_z  = -34.5;
    double initial_yaw = 0.5*M_PI;    

    // Other variables
    tf::Quaternion tf_quaternion;
    int cout_counter = 0;
    int cycle_counter = 0;
    int status = 0;
    bool turn;
    bool completed_turn;
    bool completed_round;
    bool get_height;
    float yaw_sum;
    float pre_yaw;
    float yaw_diff;
    float desired_height;
    bool completed_mission;

public:
    bool is_start = false;
    // Initialization m 
    RandomMotion(ros::NodeHandle &nh)
    {
        // ROS subscriber
        pose_gt_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/pose_gt", 20, &RandomMotion::poseGtCallback, this);
        
        // ROS publisher
        thrust0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/0/input",20);
        thrust1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/1/input",20);
        thrust2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/2/input",20);
        thrust3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/3/input",20);
        thrust4_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/4/input",20);
        thrust5_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/5/input",20);
        ref_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/mpc/reference",20);
        visual_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/visual/pose",20);
        control_input0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/0",20);
        control_input1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/1",20);
        control_input2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/2",20);
        control_input3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/3",20);
        // Initialization 
        ref.x = safety_dis;
        ref.z = initial_z;
        ref.yaw = initial_yaw;
        yaw_sum = initial_yaw;
        pre_yaw = initial_yaw;
        yaw_diff = 0;
        completed_turn = true;
        completed_round = false;
        get_height = true;
        desired_height = initial_z + safety_dis*tan(32*M_PI/180)*2;
        completed_mission = false;

    }


    // position callback
    void poseGtCallback(const nav_msgs::Odometry::ConstPtr &pose)
    {
        local_pos.x = pose->pose.pose.position.x;
        local_pos.y = pose->pose.pose.position.y;
        local_pos.z = pose->pose.pose.position.z;

        local_pos.u = pose->twist.twist.linear.x;
        local_pos.v = pose->twist.twist.linear.y;
        local_pos.w = pose->twist.twist.linear.z;

        local_pos.p = pose->twist.twist.angular.x;
        local_pos.q = pose->twist.twist.angular.y;
        local_pos.r = pose->twist.twist.angular.z;
    }


    // PID controller
    double PID(double ref, double pos, double kp, double ki, double kd) 
    {
        double error = ref-pos;
        integral += error * dt;
        double derivative = (error - prevError) / dt;
        double control_input = kp * error + ki * integral + kd * derivative;

        prevError = error;

        return control_input;
    }

    // Overall control system
    void controller()
    {
        // Set reference
        if (pre_yaw >= 0 && pos.yaw >=0)
        {
            yaw_diff = pos.yaw - pre_yaw;
        }
        else if (pre_yaw >= 0 && pos.yaw <0)
        {
            if (2*M_PI+pos.yaw-pre_yaw >= pre_yaw+abs(pos.yaw))
            {
                yaw_diff = -(pre_yaw + abs(pos.yaw));
            }
            else
            {
                yaw_diff = 2 * M_PI + pos.yaw - pre_yaw;
            }
        }
        else if (pre_yaw < 0 && pos.yaw >= 0)
        {
            if (2*M_PI-pos.yaw+pre_yaw >= abs(pre_yaw)+pos.yaw)
            {
                yaw_diff = abs(pre_yaw)+pos.yaw;
            }
            else
            {
                yaw_diff = -(2*M_PI-pos.yaw+pre_yaw);
            }
        }
        else
        {
            yaw_diff = pos.yaw - pre_yaw;
        }

        yaw_sum = yaw_sum + yaw_diff;
        pre_yaw = pos.yaw;

        // state machine
        double current_time = 0.0;
        ros::Time current_ros_time = ros::Time::now();
        current_time = current_ros_time.toSec();
        if (current_time < 1)
        {
            control_u.u1 = PID(safety_dis, pos.x, 5, 0, 0);
            control_u.u2 = 0;
            control_u.u3 = PID(initial_z, pos.z, 5, 0, 0);
            control_u.u4 = PID(0.5*M_PI, yaw_sum, 7, 0.08, 0);
        }        


        // Control allocation
        thrust0.data = (-control_u.u1 + control_u.u2 + control_u.u4)/rotor_constant;
        thrust1.data = (-control_u.u1 - control_u.u2 - control_u.u4)/rotor_constant;
        thrust2.data = (control_u.u1 + control_u.u2 - control_u.u4)/rotor_constant;
        thrust3.data = (control_u.u1 - control_u.u2 + control_u.u4)/rotor_constant;
        thrust4.data = (control_u.u3)/rotor_constant;
        thrust5.data = (control_u.u3)/rotor_constant;

        // Publish thrusts
        thrust0_pub.publish(thrust0);
        thrust1_pub.publish(thrust1);
        thrust2_pub.publish(thrust2);
        thrust3_pub.publish(thrust3);
        thrust4_pub.publish(thrust4);
        thrust5_pub.publish(thrust5);

        float yaw_ref;
        if(sin(ref.yaw) >= 0)
            {
                yaw_ref = fmod(ref.yaw,M_PI);
            }
            else{
                yaw_ref = -M_PI + fmod(ref.yaw,M_PI);
            }
        // publish reference pose
        tf2::Quaternion ref_quat;
        ref_quat.setRPY(0, 0, yaw_ref);
        geometry_msgs::Quaternion ref_quat_msg;
        tf2::convert(ref_quat, ref_quat_msg);
        ref_pose.pose.pose.position.x = ref.x;
        ref_pose.pose.pose.position.y = 0;
        ref_pose.pose.pose.position.z = ref.z;
        ref_pose.pose.pose.orientation.x = ref_quat_msg.x;
        ref_pose.pose.pose.orientation.y = ref_quat_msg.y;
        ref_pose.pose.pose.orientation.z = ref_quat_msg.z;
        ref_pose.pose.pose.orientation.w = ref_quat_msg.w;
        ref_pose.header.stamp = ros::Time::now();
        ref_pose.header.frame_id = "odom_frame";
        ref_pose.child_frame_id = "base_link";

        ref_pose_pub.publish(ref_pose);

        // publish visual pose
        tf2::Quaternion imu_quat;
        imu_quat.setRPY(0, 0, pos.yaw);
        geometry_msgs::Quaternion imu_quat_msg;
        tf2::convert(imu_quat, imu_quat_msg);
        visual_pose.pose.pose.position.x = pos.x;
        visual_pose.pose.pose.position.y = 0;
        visual_pose.pose.pose.position.z = pos.z;
        visual_pose.pose.pose.orientation.x = imu_q.x;
        visual_pose.pose.pose.orientation.y = imu_q.y;
        visual_pose.pose.pose.orientation.z = imu_q.z;
        visual_pose.pose.pose.orientation.w = imu_q.w;
        visual_pose.header.stamp = ros::Time::now();
        visual_pose.header.frame_id = "odom_frame";
        visual_pose.child_frame_id = "base_link";

        visual_pose_pub.publish(visual_pose);

        // publish conrtrol input
        control_input0.data = control_u.u1;
        control_input1.data = control_u.u2;
        control_input2.data = control_u.u3;
        control_input3.data = control_u.u4;

        control_input0_pub.publish(control_input0);
        control_input1_pub.publish(control_input1);
        control_input2_pub.publish(control_input2);
        control_input3_pub.publish(control_input3);

        // Calculate control error
        error.x = ref.x - pos.x;
        error.z = ref.z - pos.z;
        error.yaw = ref.yaw - yaw_sum;

        std::string message;

        // Mission information cout
        if(cout_counter > 2){ //reduce cout rate
                std::cout << "------------------------------------------------------------------------------------------------" << std::endl;
                std::cout << "ref_x:    " << ref.x << "\tref_z:    " << ref.z << "\tref_yaw:    " << ref.yaw << "\tcmd_u2:    " << control_u.u2 << std::endl;
                std::cout << "pos_x:    " << pos.x << "\tpos_z:    " << pos.z << "\tpos_yaw:    " << yaw_sum << std::endl;
                std::cout << "error_x   " << error.x << "\terror_z:    " << error.z << "\terror_yaw:    " << error.yaw << std::endl;
                std::cout << "x_gt:     " << local_pos.x << "\ty_gt:     " << local_pos.y << "\tz_gt:     " << local_pos.z << std::endl;
                std::cout << "u1    : " << control_u.u1 << "\tu2:    " << control_u.u2 << "\tu3:    " << control_u.u3 << "\tu4:    " << control_u.u4 << std::endl;
                std::cout << "t0:  " << thrust0.data << "\tt1:  " << thrust1.data << "\tt2:  " << thrust2.data << "\tt3:  " << thrust3.data << "\tt4:  " << thrust4.data << "\tt5:  " << thrust5.data << std::endl;
                std::cout << "no. of cycle completed:  " << cycle_counter << "  if turning?  " << turn << "  completed turning?  " << completed_turn << "  completed round?  " << completed_round << std::endl;
                std::cout << "states:  " << status << ": " << message << std::endl;
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
    ros::init(argc, argv, "bluerov2_inspection_node");
    ros::NodeHandle nh;

    RandomMotion rm(nh);
    ros::Rate loop_rate(20);
    // pcl_processor.controller();
    while(ros::ok()){
        if(rm.is_start==true)
        {
            rm.controller();
        }
        // vc.controller();
        ros::spinOnce();
        loop_rate.sleep();
    }
    // ros:spin();

    return 0;
}