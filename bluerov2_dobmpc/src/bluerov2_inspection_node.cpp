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

class VisualController
{
private:
    // ros::NodeHandle nh;

    // Subscribers & Publishers
    ros::Subscriber pcl_sub;
    ros::Subscriber pose_gt_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber pressure_sub;
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
    nav_msgs::Odometry pose_gt;
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

    // barometer parameters
    double fluid_p;
    double atomosphere_p = 101325;
    double g = 9.80665;
    double rho_salt = 1000;    

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
    VisualController(ros::NodeHandle &nh)
    {
        // ROS subscriber
        pcl_sub = nh.subscribe("/camera/depth/color/points", 20, &VisualController::pclCallback, this);
        pose_gt_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/pose_gt", 20, &VisualController::poseGtCallback, this);
        imu_sub = nh.subscribe<sensor_msgs::Imu>("/bluerov2/imu", 20, &VisualController::imuCallback, this);
        pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>("/bluerov2/pressure", 20, &VisualController::pressureCallback, this);
        
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

    // Pointcloud callback from Realsense d435
    void pclCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloudPtr);

        // Calculate the center point coordinates
        float centerX = cloudPtr->width / 2;
        float centerY = cloudPtr->height / 2;

        // Define the ROI size
        int roiSize = 40;

        // Calculate the ROI boundaries
        int roiMinX = centerX - roiSize / 2;
        int roiMaxX = centerX + roiSize / 2;
        int roiMinY = centerY - roiSize / 2;
        int roiMaxY = centerY + roiSize / 2;

        // Create a new point cloud for the ROI
        pcl::PointCloud<pcl::PointXYZ>::Ptr roiCloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Iterate over the points within the ROI boundaries
        for (int y = roiMinY; y < roiMaxY; y++)
        {
            for (int x = roiMinX; x < roiMaxX; x++)
            {
                // Add the point to the ROI point cloud
                roiCloud->push_back(cloudPtr->at(x, y));
            }
        }

        // calculate average distance
        double average_distance = calculateAverageDistance(roiCloud);
        pos.x = average_distance;
        if(pos.x > 0+buffer)
        {
            is_start = true;
        }

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

    void imuCallback(const sensor_msgs::Imu::ConstPtr &imu)
    {
        // get angle phi, theta, psi
        tf::quaternionMsgToTF(imu->orientation,tf_quaternion);
        imu_q.w = imu->orientation.w;
        imu_q.x = imu->orientation.x;
        imu_q.y = imu->orientation.y;
        imu_q.z = imu->orientation.z;
        tf::Matrix3x3(tf_quaternion).getRPY(local_euler.phi, local_euler.theta, local_euler.psi);
        pos.yaw = local_euler.psi;

    }

    void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr &pressure)
    {
        fluid_p = (pressure->fluid_pressure)*1000;
        pos.z = -(fluid_p-atomosphere_p)/(rho_salt*g);
        
    }

    // calculate average distance between camera and bridge pier based on ROI
    double calculateAverageDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        double totalDistance = 0.0;
        size_t numPoints = 0;
        for (const auto& point : cloud->points)
        {
            // Avoid zero or nan value
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
                point.x != 0.0 && point.y != 0.0 && point.z != 0.0)
            {
                double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
                totalDistance += distance;
                numPoints++;
            }
        }
        double averageDistance = (numPoints > 0) ? (totalDistance / numPoints) : 0.0;
        return averageDistance;
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

        // // PID weighting
        // double current_time = 0.0;
        // ros::Time current_ros_time = ros::Time::now();
        // current_time = current_ros_time.toSec();
        // ref.yaw = 0.5*M_PI;
        // if (current_time < 5.0) {
        //     ref.yaw = 0.5 * M_PI;
        // } else if (current_time >= 5.0 && current_time <= 30.0) {
        //     ref.yaw = M_PI;
        // }

        // if (current_time < 5.0) {
        //     control_u.u1 = PID(ref.x, pos.x, 5, 0, 0);
        //     control_u.u2 = 0;
        //     control_u.u3 = PID(initial_z, pos.z, 5, 0, 0);
        //     control_u.u4 = PID(0.5*M_PI, yaw_sum, 7, 0.08, 0);
        // } else if (current_time >= 5.0 && current_time < 10.0) {
        //     ref.x = safety_dis+0.5;
        //     control_u.u1 = PID(ref.x, pos.x, 5, 0, 0);
        //     control_u.u2 = 0;
        //     control_u.u3 = PID(initial_z, pos.z, 5, 0, 0);
        //     control_u.u4 = PID(0.5*M_PI, yaw_sum, 7, 0.08, 0);
        // }
        // else if (current_time >= 10.0 && current_time <= 15.0) {
        //     ref.x = safety_dis+0.5;
        //     ref.z = initial_z + 5;
        //     // ref.yaw = M_PI;
        //     control_u.u1 = PID(ref.x, pos.x, 5, 0, 0);
        //     control_u.u2 = 0;
        //     control_u.u3 = PID(ref.z, pos.z, 5, 0, 0);
        //     control_u.u4 = PID(0.5*M_PI, yaw_sum, 7, 0.08, 0);
        // }
        // else {
        //     ref.x = 0;
        //     ref.yaw = M_PI;
        //     control_u.u1 = 0;
        //     control_u.u2 = 0;
        //     control_u.u3 = PID(ref.z, pos.z, 5, 0, 0);
        //     control_u.u4 = PID(ref.yaw, yaw_sum, 7, 0.08, 0);
        // }

        // control_u.u3 = PID(ref.z, pos.z, 5, 0, 0);
        // control_u.u4 = PID(ref.yaw, yaw_sum, 5, 0.15, 0);

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
        else
        {
            if (completed_mission == false)
            {
                // when UUV faces to one side of bridge pier: Keep safety distance to bridge pier, move laterally
                if (pos.x >= safety_dis-buffer && pos.x <= safety_dis+buffer && completed_turn == true && completed_round == false)
                {
                    status = 0;
                    ref.x = safety_dis;
                    // ref.z = initial_z;
                    // ref.yaw = initial_yaw;
                    control_u.u1 = PID(ref.x, pos.x, 5, 0, 0);
                    control_u.u3 = PID(ref.z, pos.z, 5, 0, 0);
                    control_u.u4 = PID(ref.yaw, yaw_sum, 7, 0.08, 0);
                    control_u.u2 = -4; 
                    completed_turn = true;  
                    turn = false;
                    get_height = true;
                }
                // when UUV approaches to the edge: stop control in x direction, move slowly laterally
                else if (pos.x > safety_dis+buffer && completed_turn == true)
                {
                    status = 1;
                    control_u.u1 = 0;
                    control_u.u3 = PID(ref.z, pos.z, 5, 0, 0);
                    control_u.u4 = PID(ref.yaw, yaw_sum, 7, 0.08, 0);
                    control_u.u2 = -2;  
                }
                // when UUV reaches to the edge of bridge pier: turn 90 degrees in yaw direction
                else if (pos.x < 0.1 && turn == false)
                {
                    completed_turn = false;
                    // std::cout << "turn 90 degrees !!" << std::endl;
                    status = 2;
                    // turn 90 degrees
                    ref.yaw = ref.yaw + 0.5*M_PI;
                    control_u.u1 = 0;
                    control_u.u3 = PID(ref.z, pos.z, 5, 0, 0);
                    control_u.u4 = PID(ref.yaw, yaw_sum, 7, 0.08, 0);
                    control_u.u2 = 0;
                    // if (fabs(ref.yaw - yaw_sum) < 0.1)
                    // {
                    //     completed_turn = true;
                    // }
                    turn = true;
                }

                // turning until get to desired orientation
                else if (turn == true && completed_turn == false)
                {
                    status = 2;
                    control_u.u1 = 0;
                    control_u.u3 = PID(ref.z, pos.z, 5, 0, 0);
                    control_u.u4 = PID(ref.yaw, yaw_sum, 7, 0.08, 0);
                    control_u.u2 = 0;
                    if (fabs(ref.yaw - yaw_sum) < 0.1)
                    {
                        completed_turn = true;
                        if (fabs(pos.yaw - initial_yaw) < 0.2)
                        {
                            completed_round = true;
                            cycle_counter ++;
                            if (fabs(ref.z - desired_height) < 0.1)
                            {
                                completed_mission = true;
                            }
                        }
                        else
                        {
                            completed_round = false;
                        }
                    }
                }
                // when UUV completes turning: finding the next bridge pier
                else if (pos.x < 0.1 && completed_turn == true)
                {
                    // turn = false;
                    status = 3;
                    control_u.u1 = 1;
                    control_u.u2 = -4;
                    control_u.u3 = PID(ref.z, pos.z, 5, 0, 0);
                    control_u.u4 = PID(ref.yaw, yaw_sum, 7, 0.08, 0);
                }
                // when UUV finds the next bridge pier and go back to initial position: completed one round, move vertically
                else if (pos.x >= safety_dis-buffer && pos.x <= safety_dis+buffer && completed_turn == true && completed_round == true)
                {
                    status = 4;
                    if (get_height == true)
                    {
                        ref.z = ref.z + safety_dis*tan(32*M_PI/180)*2;
                        ref.x = safety_dis;
                        control_u.u1 = 0;
                        control_u.u2 = 0;
                        control_u.u3 = PID(ref.z, pos.z, 5, 0, 0);
                        control_u.u4 = PID(ref.yaw, yaw_sum, 7, 0.08, 0);
                        get_height = false;
                    }
                    else
                    {
                        ref.x = safety_dis;
                        control_u.u1 = 0;
                        control_u.u2 = 0;
                        control_u.u3 = PID(ref.z, pos.z, 5, 0, 0);
                        control_u.u4 = PID(ref.yaw, yaw_sum, 7, 0.08, 0);
                        if (fabs(ref.z - pos.z) < 0.25)
                        {
                            completed_round = false;
                        }
                    }
                    // status = 4;
                    // ref.z = ref.z + safety_dis*tan(32*M_PI/180)*2;
                    // control_u.u1 = PID(ref.x, pos.x, 5, 0, 0);
                    // control_u.u2 = 0;
                    // control_u.u3 = PID(ref.z, pos.z, 5, 0, 0);
                    // control_u.u4 = PID(ref.yaw, yaw_sum, 7, 0.08, 0);
                    // completed_round = false;
                }
                else
                {
                    std::cout << "state machine failed!" << std::endl;
                    status = 2;
                    // ref.x = pos.x;
                    // ref.z = pos.z;
                    // ref.yaw = pos.yaw;
                    control_u.u1 = 0;
                    control_u.u3 = PID(ref.z, pos.z, 5, 0, 0);
                    control_u.u4 = PID(ref.yaw, yaw_sum, 7, 0.08, 0);
                    control_u.u2 = 0;
                    if (fabs(ref.yaw - yaw_sum) < 0.1)
                    {
                        completed_turn = true;
                    }
                }
            }
            else
            {
                status = 5;
                control_u.u1 = 0;
                control_u.u3 = PID(ref.z, pos.z, 5, 0, 0);
                control_u.u4 = PID(ref.yaw, yaw_sum, 7, 0.08, 0);
                control_u.u2 = 0;
            }
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

        if (status == 0) 
        {
            message = "Keep safety distance to bridge pier, move laterally";
        } 
        else if (status == 1) 
        {
            message = "Stop control in x direction, move slowly laterally";
        } 
        else if (status == 2) 
        {
            message = "Turn 90 degrees in yaw direction";
        } 
        else if (status == 3) 
        {
            message = "Completed turning, finding the next bridge pier";
        } 
        else if (status == 4)
        {
            message = "Completed one cycle, move vertically";
        }
        else if (status == 5)
        {
            message = "Mission completed!";
        }
        else 
        {
        message = "Stay with previous command";
        }
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

    VisualController vc(nh);
    ros::Rate loop_rate(20);
    // pcl_processor.controller();
    while(ros::ok()){
        if(vc.is_start==true)
        {
            vc.controller();
        }
        // vc.controller();
        ros::spinOnce();
        loop_rate.sleep();
    }
    // ros:spin();

    return 0;
}