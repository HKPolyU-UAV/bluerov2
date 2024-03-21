#include <ros_utilities/ros_utilities.h>

static std::deque<Eigen::Vector3d> accl_buff;
static std::deque<Eigen::Vector3d> gyro_buff;
static std::deque<nav_msgs::Odometry::ConstPtr> gt_buff;
static bool calibrate_start = false, got_imu = false, got_gt = false;
static double starting_time;

static RosUtilities tool;

static double max_static_accl_var;
static double max_static_gyro_var;
const static double gravity_constant = -9.81;

void bias_calculate();
bool try_init();

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(!got_imu)
        got_imu = true;

    accl_buff.push_back(
        Eigen::Vector3d(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        )
    );

    ros::shutdown();
}

void gt_pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(!got_gt)
        got_gt = true;
    
    gt_buff.push_back(msg);
}

void mainspin_cb(const ros::TimerEvent& e)
{
    if(try_init())
        ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "disturbance");
    ros::NodeHandle nh("~");

    nh.getParam("max_bias_accl_var", max_static_accl_var);
    nh.getParam("max_bias_gyro_var", max_static_gyro_var);

    
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("/bluerov2/imu", 1, &imu_callback);
    ros::Subscriber pose_gt_sub = nh.subscribe<nav_msgs::Odometry>
            ("/bluerov2/pose_gt", 1, &gt_pose_callback);
    ros::Timer mainspin_timer = nh.createTimer(
        ros::Duration(1.0/50.0),
        &mainspin_cb
    );
    
    ros::spin();

    bias_calculate();

    return 0;
}

void bias_calculate()
{
    std::cout<<"gan"<<std::endl;
}

bool try_init()
{
    if(
        got_imu 
        && 
        got_gt 
        && 
        !calibrate_start
    )
    {
        starting_time = ros::Time::now().toSec();
        calibrate_start = true;
    }

    if(
        (
            starting_time - 
            ros::Time::now().toSec()
        )
        <
        ros::Duration(10).toSec()
    )
        return false;
}