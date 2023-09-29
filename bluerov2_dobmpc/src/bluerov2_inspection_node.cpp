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

    // ROS message variables
    nav_msgs::Odometry pose_gt;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust0;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust2;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust3;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust4;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust5;

    // PID parameters
    double kp = 10;  
    double ki = 0;  
    double kd = 0; 
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
    controlInputs pid_u;
    controlInputs openLoop_u;
    pose local_pos;
    double dt = 0.05;
    double rotor_constant = 0.026546960744430276;
    double safety_dis = 1.79;
    double buffer = 0.3;
    double sway = -4;

    // barometer parameters
    double fluid_p;
    double atomosphere_p = 101325;
    double g = 9.80665;
    double rho_salt = 1000;    

    // Other variables
    tf::Quaternion tf_quaternion;
    int cout_counter = 0;

public:
    // Initialization 
    VisualController(ros::NodeHandle &nh)
    {
        // ROS subscriber
        pcl_sub = nh.subscribe("/camera/depth/color/points", 20, &VisualController::pclCallback, this);
        pose_gt_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/pose_gt", 20, &VisualController::poseGtCallback, this);
        imu_sub = nh.subscribe<sensor_msgs::Imu>("/bluerov2/imu", 20, &VisualController::imuCallback, this);
        pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>("/bluerov2/pressure", 20, &VisualController::pressureCallback, this);
        
        // ROS publisher
        // thrust0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/0/input",20);
        // thrust1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/1/input",20);
        // thrust2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/2/input",20);
        // thrust3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/3/input",20);
        // thrust4_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/4/input",20);
        // thrust5_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/5/input",20);
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

        // if(cout_counter > 2){ //reduce cout rate
        //     std::cout << "pos.x:    " << pos.x << std::endl;
        //     cout_counter = 0;
        //     }
        // else{
        //     cout_counter++;
        //     }
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
        tf::Matrix3x3(tf_quaternion).getRPY(local_euler.phi, local_euler.theta, local_euler.psi);
        pos.yaw = local_euler.psi;

        // if(cout_counter > 2){ //reduce cout rate
        //     std::cout << "pos.yaw:    " << pos.yaw << std::endl;
        //     cout_counter = 0;
        //     }
        // else{
        //     cout_counter++;
        //     }
    }

    void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr &pressure)
    {
        fluid_p = (pressure->fluid_pressure)*1000;
        pos.z = -(fluid_p-atomosphere_p)/(rho_salt*g);
        
        // if(cout_counter > 2){ //reduce cout rate
        //     std::cout << "pos.z:    " << pos.z << std::endl;
        //     cout_counter = 0;
        //     }
        // else{
        //     cout_counter++;
        //     }
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
    double PID(double ref, double pos, double dt) 
    {
        double error = pos - ref;
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
        ref.x = safety_dis;
        ref.z = -34.5;
        ref.yaw = 0.5*M_PI;
        openLoop_u.u2 = 0;

        // Call PID to get control inputs
        pid_u.u1 = PID(ref.x, pos.x, dt);
        pid_u.u3 = PID(ref.z, pos.z, dt);
        pid_u.u4 = PID(ref.yaw, pos.yaw, dt);

        // Control allocation
        thrust0.data = (-pid_u.u1 + openLoop_u.u2 + pid_u.u4)/rotor_constant;
        thrust1.data = (-pid_u.u1 - openLoop_u.u2 - pid_u.u4)/rotor_constant;
        thrust2.data = (pid_u.u1 + openLoop_u.u2 - pid_u.u4)/rotor_constant;
        thrust3.data = (pid_u.u1 - openLoop_u.u2 + pid_u.u4)/rotor_constant;
        thrust4.data = (-pid_u.u3)/rotor_constant;
        thrust5.data = (-pid_u.u3)/rotor_constant;

        // Publish thrusts
        // thrust0_pub.publish(thrust0);
        // thrust1_pub.publish(thrust1);
        // thrust2_pub.publish(thrust2);
        // thrust3_pub.publish(thrust3);
        // thrust4_pub.publish(thrust4);
        // thrust5_pub.publish(thrust5);

        // Calculate control error
        error.x = pos.x - ref.x;
        error.z = pos.z - ref.z;
        error.yaw = pos.yaw - ref.yaw;

        // Mission information cout
        if(cout_counter > 2){ //reduce cout rate
                std::cout << "------------------------------------------------------------------------------------------------" << std::endl;
                std::cout << "ref_x:    " << ref.x << "\tref_z:    " << ref.z << "\tref_yaw:    " << ref.yaw << std::endl;
                std::cout << "pos_x:    " << pos.x << "\tpos_z:    " << pos.z << "\tpos_yaw:    " << pos.yaw << std::endl;
                std::cout << "error_x   " << error.x << "\terror_z:    " << error.z << "\terror_yaw:    " << error.yaw << std::endl;
                std::cout << "x_gt:     " << local_pos.x << "\ty_gt:     " << local_pos.y << "\tz_gt:     " << local_pos.z << std::endl;
                std::cout << "u1    : " << pid_u.u1 << "\tu2:    " << openLoop_u.u2 << "\tu3:    " << pid_u.u3 << "\tu4:    " << pid_u.u4 << std::endl;
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
    ros::init(argc, argv, "bluerov2_inspection_node");
    ros::NodeHandle nh;

    VisualController vc(nh);
    ros::Rate loop_rate(20);
    // pcl_processor.controller();
    while(ros::ok()){
        vc.controller();
        ros::spinOnce();
        loop_rate.sleep();
    }
    // ros:spin();

    return 0;
}