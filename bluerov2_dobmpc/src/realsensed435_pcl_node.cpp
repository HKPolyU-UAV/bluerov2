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

class PointCloudProcessor
{
private:
    tf::Quaternion tf_quaternion;
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Subscriber pose_gt_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber pressure_sub;
    nav_msgs::Odometry pose_gt;
    double kp;  // Proportional gain
    double ki;  // Integral gain
    double kd;  // Derivative gain
    double prevError;
    double integral;
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
    Euler local_euler;
    state ref;
    state pos;
    float fluid_p;
    float atomosphere_p = 101325;
    float g = 9.80665;
    float rho_salt = 1000;

public:
    // Initialization 
    PointCloudProcessor(ros::NodeHandle &nh)
    {
        pcl_sub = nh.subscribe("/camera/depth/color/points", 20, &PointCloudProcessor::pclCallback, this);
        pose_gt_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/pose_gt", 20, &PointCloudProcessor::poseGtCallback, this);
        imu_sub = nh.subscribe<sensor_msgs::Imu>("/bluerov2/imu", 20, &PointCloudProcessor::imuCallback, this);
        pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>("/bluerov2/pressure", 20, &PointCloudProcessor::pressureCallback, this);
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
        // ROS_INFO("Received %ld points in ROI.", roiCloud->points.size());

        // calculate average distance
        double average_distance = calculateAverageDistance(roiCloud);
        pos.x = average_distance;
        ROS_INFO("Average distance to ROI: %f", average_distance);
    }

    // position callback
    void poseGtCallback(const nav_msgs::Odometry::ConstPtr &pose)
    {
        pose_gt.pose.pose.position.x = pose->pose.pose.position.x;
        pose_gt.pose.pose.position.y = pose->pose.pose.position.y;
        pose_gt.pose.pose.position.z = pose->pose.pose.position.z;

        pose_gt.twist.twist.linear.x = pose->twist.twist.linear.x;
        pose_gt.twist.twist.linear.y = pose->twist.twist.linear.y;
        pose_gt.twist.twist.linear.z = pose->twist.twist.linear.z;

        pose_gt.twist.twist.angular.x = pose->twist.twist.angular.x;
        pose_gt.twist.twist.angular.y = pose->twist.twist.angular.y;
        pose_gt.twist.twist.angular.z = pose->twist.twist.angular.z;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &imu)
    {
        // get angle phi, theta, psi
        tf::quaternionMsgToTF(imu->orientation,tf_quaternion);
        tf::Matrix3x3(tf_quaternion).getRPY(local_euler.phi, local_euler.theta, local_euler.psi);
        pos.yaw = local_euler.psi;
    }

    void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr &pressure)
    {
        fluid_p = (pressure->fluid_pressure)*1000;
        ROS_INFO("pressure: %f", fluid_p);
        pos.z = (fluid_p-atomosphere_p)/(rho_salt*g);
        ROS_INFO("depth: %f", pos.z);
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

    // // PID controller
    // double PID(double ref, double pos, double dt) 
    // {
    //     double error = ref - pos;
    //     integral += error * dt;
    //     double derivative = (error - prevError) / dt;
    //     double control_input = kp * error + ki * integral + kd * derivative;

    //     prevError = error;

    //     return control_input;
    // }

    // Overall control system
    // void controller()
    // {

    // }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsensed435_pcl_node");
    ros::NodeHandle nh;

    PointCloudProcessor pcl_processor(nh);

    ros::spin();

    return 0;
}