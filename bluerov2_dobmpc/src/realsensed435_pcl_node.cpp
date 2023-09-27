#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>

class PointCloudProcessor
{
private:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Subscriber pose_gt_sub;
    nav_msgs::Odometry pose_gt;

public:
    PointCloudProcessor(ros::NodeHandle &nh)
    {
        pcl_sub = nh.subscribe("/camera/depth/color/points", 100, &PointCloudProcessor::pclCallback, this);
        pose_gt_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/pose_gt", 20, &PointCloudProcessor::poseGtCallback, this);
    }

    void pclCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloudPtr);

        // double average_distance = calculateAverageDistance(cloudPtr);

        // ROS_INFO("Average distance to ROI: %f", average_distance);

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
        ROS_INFO("Received %ld points in ROI.", roiCloud->points.size());

        // calculate average distance
        double average_distance = calculateAverageDistance(roiCloud);
        ROS_INFO("Average distance to ROI: %f", average_distance);
    }

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

    
    double calculateAverageDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        double totalDistance = 0.0;
        size_t numPoints = cloud->points.size();

        for (const auto& point : cloud->points)
        {
            double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            totalDistance += distance;
        }

        double averageDistance = (numPoints > 0) ? (totalDistance / numPoints) : 0.0;
        return averageDistance;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsensed435_pcl_node");
    ros::NodeHandle nh;

    PointCloudProcessor pcl_processor(nh);

    ros::spin();

    return 0;
}