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

class pointcloud_pub_sub
{
    private:
        ros::Subscriber pcl_sub;
        std::string pointcloud_topic = "/camera/depth/color/points";
        ros::Subscriber pose_gt_sub;
        nav_msgs::Odometry pose_gt;

    public:

    pointcloud_pub_sub(ros::NodeHandle& nh)
    {
        pcl_sub = nh.subscribe(pointcloud_topic, 10, &pointcloud_pub_sub::callback, this);
        pose_gt_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/pose_gt", 20, &pointcloud_pub_sub::pose_gt_cb, this);
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

        //do stuff
        ROS_INFO("received %ld points", temp_cloud->points.size());

    }

    void pose_gt_cb(const nav_msgs::Odometry::ConstPtr& pose)
    {
        // get linear position x, y, z
        pose_gt.pose.pose.position.x = pose->pose.pose.position.x;
        pose_gt.pose.pose.position.y = pose->pose.pose.position.y;
        pose_gt.pose.pose.position.z = pose->pose.pose.position.z;

        // get linear velocity u, v, w
        pose_gt.twist.twist.linear.x = pose->twist.twist.linear.x;
        pose_gt.twist.twist.linear.y = pose->twist.twist.linear.y;
        pose_gt.twist.twist.linear.z = pose->twist.twist.linear.z;

        // get angular velocity p, q, r
        pose_gt.twist.twist.angular.x = pose->twist.twist.angular.x;
        pose_gt.twist.twist.angular.y = pose->twist.twist.angular.y;
        pose_gt.twist.twist.angular.z = pose->twist.twist.angular.z;
        // std::cout << "position x: " << pose_gt.pose.pose.position.x << std::endl;
    }

    // void test()
    // {
    //     std::cout << "position x: " << pose_gt.pose.pose.position.x << std::endl;
    // }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsensed435_pcl_node");
    ros::NodeHandle nh;

    pointcloud_pub_sub pcl_pub_sub(nh);
    ros::Rate loop_rate(10);
    while(ros::ok()){
        // pcl_pub_sub.test();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}