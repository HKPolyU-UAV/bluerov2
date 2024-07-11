#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
    ros::Publisher control_input0_pub;
    ros::Publisher control_input1_pub;
    ros::Publisher control_input2_pub;
    ros::Publisher control_input3_pub;

    // ROS message variables
    nav_msgs::Odometry pose_gt;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust0;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust2;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust3;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust4;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust5;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input0;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input2;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input3;


    // System variables
    struct Euler{
        double phi;
        double theta;
        double psi;
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
    controlInputs control_u;
    pose local_pos;
    double dt = 0.05;
    double rotor_constant = 0.026546960744430276;  

    // Other variables
    tf::Quaternion tf_quaternion;
    int cout_counter = 0;
    int random_dof;
    double random_input;

public:
    // bool is_start = false;
    // Initialization 
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
        
        control_input0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/0",20);
        control_input1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/1",20);
        control_input2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/2",20);
        control_input3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/3",20);

        random_dof = 1;
        double random_input = 0.0;
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

        // get angle phi, theta, psi
        tf::quaternionMsgToTF(pose->pose.pose.orientation,tf_quaternion);
        tf::Matrix3x3(tf_quaternion).getRPY(local_euler.phi, local_euler.theta, local_euler.psi);
    }


    void single_dof_motion(int dof)
    {
        srand(time(0));
        random_input = -15 + static_cast<double>(rand()) / RAND_MAX * (15 - (-15));
        if(dof == 1)
        {
            control_u.u1 = random_input;
            control_u.u2 = 0;
            control_u.u3 = 0;
            control_u.u4 = 0;
        }
        else if(dof == 2)
        {
            control_u.u1 = 0;
            control_u.u2 = random_input;
            control_u.u3 = 0;
            control_u.u4 = 0;
        }
        else if(dof == 3)
        {
            if(local_pos.z > -5 && random_input > 0)
            {
                random_input = -random_input;
            }
            control_u.u1 = 0;
            control_u.u2 = 0;
            control_u.u3 = random_input;
            control_u.u4 = 0;
        }
        else if(dof == 4)
        {
            control_u.u1 = 0;
            control_u.u2 = 0;
            control_u.u3 = 0;
            control_u.u4 = random_input;
        }
        else
        {
            ROS_WARN("Invalid dof: %d", dof);
        }
        
    }

    // Overall control system
    void controller()
    {
        srand(time(0));     // Seed the random number generator
        random_dof = 1 + rand() % 4; // Generate a random real number between 1 and 4
        single_dof_motion(random_dof);
               

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


        // publish conrtrol input
        control_input0.data = control_u.u1;
        control_input1.data = control_u.u2;
        control_input2.data = control_u.u3;
        control_input3.data = control_u.u4;

        control_input0_pub.publish(control_input0);
        control_input1_pub.publish(control_input1);
        control_input2_pub.publish(control_input2);
        control_input3_pub.publish(control_input3);


        // Mission information cout
        if(cout_counter > 2){ //reduce cout rate
                std::cout << "------------------------------------------------------------------------------------------------" << std::endl;
                std::cout << "random DOF:    " << random_dof << "random input:    " << random_input << std::endl;
                std::cout << "x_gt:     " << local_pos.x << "\ty_gt:     " << local_pos.y << "\tz_gt:     " << local_pos.z << "\tpsi_gt:     " << local_euler.psi << std::endl;
                std::cout << "u1    : " << control_u.u1 << "\tu2:    " << control_u.u2 << "\tu3:    " << control_u.u3 << "\tu4:    " << control_u.u4 << std::endl;
                std::cout << "t0:  " << thrust0.data << "\tt1:  " << thrust1.data << "\tt2:  " << thrust2.data << "\tt3:  " << thrust3.data << "\tt4:  " << thrust4.data << "\tt5:  " << thrust5.data << std::endl;
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
    ros::init(argc, argv, "bluerov2_random_node");
    ros::NodeHandle nh;

    RandomMotion rm(nh);
    ros::Rate loop_rate(20);
    // pcl_processor.controller();
    while(ros::ok()){
        rm.controller();
        ros::spinOnce();
        loop_rate.sleep();
    }
    // ros:spin();

    return 0;
}