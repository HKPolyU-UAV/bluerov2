#include <ros_utilities/ros_utilities.h>

static geometry_msgs::Point wrench_gt;

static nav_msgs::Odometry est_x_data;
static nav_msgs::Odometry gt_x_data;
static geometry_msgs::PointStamped ref_x_data;

static geometry_msgs::Point ref_pt;
// static geometry_msgs::Point ref_pt;

static double starting_time = 0;
static std::string log_file_wrench;
static std::string log_file_traj;

static bool log_start = false;
static bool path_start = false;
static double path_starting_time = 0;

static RosUtilities tool;

void wrench_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    wrench_gt = *msg;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    est_x_data = *msg;
}

void starto_callback(const std_msgs::Bool::ConstPtr& msg)
{
    path_start = true;
}

void odom_gt_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    gt_x_data = *msg;
    Eigen::Vector3d velo_B = tool.posemsg_to_SE3(msg->pose.pose).rotationMatrix().inverse() * tool.twistmsg_to_velo(msg->twist.twist).head(3);
    
    // gt_x_data.twist.twist.linear.x = velo_B.x();
    // gt_x_data.twist.twist.linear.y = velo_B.y();
    // gt_x_data.twist.twist.linear.z = velo_B.z();
}

void xi_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    // std::cout<<"haha"<<std::endl;
    if(!log_start)
    {
        log_start = true;
        starting_time = ros::Time::now().toSec();
    }

    // std::cout<<gt_x_data.twist.twist.linear.x<<std::endl<<std::endl;;
    Eigen::Vector3d est_R = tool.q2rpy(
        tool.posemsg_to_SE3(est_x_data.pose.pose).unit_quaternion()
    );
    Eigen::Vector3d gt_R  = tool.q2rpy(
        tool.posemsg_to_SE3(gt_x_data.pose.pose).unit_quaternion()
    );

    std::ofstream save(log_file_wrench, std::ios::app);
    // save<<"t,x_est,y_est,z_est,x_gt,y_gt,z_gt,"<<std::endl;
    // px_est, py_est, pz_est, vx_est, vy_est, vz_est, px_gt, py_gt, pz_gt, vx_gt, vy_gt, vz_gt
    save << ros::Time::now().toSec() - starting_time << "," 
         << msg->x / 10<< "," 
         << msg->y / 10<< "," 
         << msg->z / 10<< ","                                                
         << wrench_gt.x / 10 << "," 
         << wrench_gt.y / 10<< "," 
         << wrench_gt.z / 10<< "," 
         << est_x_data.pose.pose.position.x << ","
         << est_x_data.pose.pose.position.y << ","
         << est_x_data.pose.pose.position.z << ","
         << est_x_data.twist.twist.linear.x << ","
         << est_x_data.twist.twist.linear.y << ","
         << est_x_data.twist.twist.linear.z << ","
         << gt_x_data.pose.pose.position.x << ","
         << gt_x_data.pose.pose.position.y << ","
         << gt_x_data.pose.pose.position.z << ","
         << gt_x_data.twist.twist.linear.x << ","
         << gt_x_data.twist.twist.linear.y << ","
         << gt_x_data.twist.twist.linear.z << ","
         << est_R(0) << ","
         << est_R(1) << ","
         << est_R(2) << ","
         << gt_R(0) << ","
         << gt_R(1) << ","
         << gt_R(2) << ","
         << std::endl;
    save.close();

    // if((ros::Time::now().toSec() - starting_time) > ros::Duration(45).toSec())
    //     ros::shutdown();
}

void ref_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if(!path_start)
    {
        path_starting_time = ros::Time::now().toSec();
        return;
    }
        
    // std::cout<<"?"<<std::endl;
    ref_x_data = *msg;
    
    std::ofstream save(log_file_traj, std::ios::app);
    // save<<"t,px_gt,py_gt,pz_gt,px_ref,py_ref,pz_ref,"<<std::endl;

    save << ros::Time::now().toSec() - starting_time << "," 
         << gt_x_data.pose.pose.position.x << ","
         << gt_x_data.pose.pose.position.y << ","
         << gt_x_data.pose.pose.position.z << ","
         << msg->point.x << ","
         << msg->point.y << ","
         << msg->point.z << ","
         << std::endl;
    save.close();

    if((ros::Time::now().toSec() - path_starting_time) > ros::Duration(30).toSec())
        ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "disturbance");
    ros::NodeHandle nh("~");

    std::string folder_name;
    nh.getParam("log_file", folder_name);
    
    ros::Subscriber wrench_sub = nh.subscribe<geometry_msgs::Point>
            ("/current_wrench", 1, &wrench_callback);
    ros::Subscriber xi_sub = nh.subscribe<geometry_msgs::Point>
            ("/xi", 1, &xi_callback);

    ros::Subscriber est_x_sub = nh.subscribe<nav_msgs::Odometry>
            ("/est_x", 1, &odom_callback);

    ros::Subscriber odom_gt_sub = nh.subscribe<nav_msgs::Odometry>
            ("/bluerov2/pose_gt", 1, &odom_gt_callback);
    
    ros::Subscriber ref_sub = nh.subscribe<geometry_msgs::PointStamped>
            ("/bluerov2/mpc/reference", 1, &ref_callback);

    ros::Subscriber start_sub = nh.subscribe<std_msgs::Bool>
            ("/path_start", 1, starto_callback);

    log_file_wrench = folder_name + "wrench.csv";
    log_file_traj = folder_name + "traj.csv";
    
    if (std::ifstream(log_file_wrench)) 
    {
        remove( 
            log_file_wrench.c_str()
        );
    }

    if (std::ifstream(log_file_traj)) 
    {
        remove( 
            log_file_traj.c_str()
        );
    }


    std::ofstream save(log_file_wrench, std::ios::app);
    save<<"t,x_est,y_est,z_est,x_gt,y_gt,z_gt,px_est,py_est,pz_est,vx_est,vy_est,vz_est,px_gt,py_gt,pz_gt,vx_gt,vy_gt,vz_gt,rr_est,rp_est,ry_est,rr_gt,rp_gt,ry_gt,"<<std::endl;
    save.close();


    save = std::ofstream(log_file_traj, std::ios::app);
    save<<"t,px_gt,py_gt,pz_gt,px_ref,py_ref,pz_ref,"<<std::endl;
    save.close();

    ros::spin();

    return 0;
}