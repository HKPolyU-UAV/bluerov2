#include <ros_utilities/ros_utilities.h>
// #include <cmath>

static std::deque<Eigen::Vector3d> accl_buff;
static std::deque<Eigen::Vector3d> gyro_buff;
static std::deque<nav_msgs::Odometry::ConstPtr> gt_buff;
static bool calibrate_start = false, got_imu = false, got_gt = false;
static double starting_time = INFINITY;

static RosUtilities tool;

static double max_static_accl_var;
static double max_static_gyro_var;
static std::string save_bias_path;
static double time_of_collection;

const static double g_constant = 9.81;

void bias_calculate();
bool try_init();
void update_accl_buffer(const Eigen::Vector3d& g_vec);
void save_bias(
    const Eigen::Vector3d accl_bias_,
    const Eigen::Vector3d accl_gyro_
);

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

    gyro_buff.push_back(
        Eigen::Vector3d(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        )
    );
}

void gt_pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(!got_gt)
        got_gt = true;
    
    gt_buff.push_back(msg);
}

void mainspin_cb(const ros::TimerEvent& e)
{
    ROS_GREEN_STREAM("COLLECT IMU INIT DATA!");

    if(try_init())
    {
        ros::shutdown();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "disturbance");
    ros::NodeHandle nh("~");

    nh.getParam("max_bias_accl_var", max_static_accl_var);
    nh.getParam("max_bias_gyro_var", max_static_gyro_var);
    nh.getParam("save_bias_path", save_bias_path);
    nh.getParam("time_of_collection", time_of_collection);

    std::cout<<max_static_accl_var<<std::endl;
    std::cout<<max_static_gyro_var<<std::endl;
    std::cout<<save_bias_path<<std::endl;
    std::cout<<time_of_collection<<std::endl;

    Eigen::Matrix4d temp;
    temp.setIdentity();
    temp.block<3,1>(0,3) = Eigen::Vector3d::Ones();
    std::cout<<temp<<std::endl<<std::endl;;
    std::cout<< temp.inverse() << std::endl<<std::endl;
    std::cout<< temp.transpose()<<std::endl<<std::endl;


    ros::shutdown();

    
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
    Eigen::Vector3d accl_mean_, accl_cov_;
    tool.ComputeMean(accl_buff, accl_mean_);
    tool.ComputeVariance(accl_buff, accl_cov_);

    Eigen::Vector3d g_vector_ = - accl_mean_ / accl_mean_.norm() * g_constant;
    update_accl_buffer(g_vector_);

    tool.ComputeMean(accl_buff, accl_mean_);
    tool.ComputeVariance(accl_buff, accl_cov_);

    std::cout<<"\nACCL BIAS: \n";
    std::cout<<accl_mean_<<std::endl;

    Eigen::Vector3d gyro_mean_, gyro_cov_;
    tool.ComputeMean(gyro_buff, gyro_mean_);
    tool.ComputeMean(gyro_buff, gyro_cov_);

    std::cout<<"GYRO BIAS: \n";
    std::cout<<gyro_mean_<<std::endl<<std::endl;

    if(
        accl_cov_.norm() > max_static_accl_var
        ||
        gyro_cov_.norm() > max_static_gyro_var
    )
    {
        std::cout<<""<<std::endl;
        RED_STREAM("FAIL TO SAVE BIAS COS COVARIANCE TOO BIG!");
        // ROS_RED_STREAM();
        return;
    }

    save_bias(accl_mean_, gyro_mean_);
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
            ros::Time::now().toSec()
            -
            starting_time
        )
        <
        ros::Duration(time_of_collection).toSec()
    )
        return false;
    

    return true;
}

void update_accl_buffer(
    const Eigen::Vector3d& g_vec_
)
{
    for(auto& what : accl_buff)
        what = what + g_vec_;
}

void save_bias(
    const Eigen::Vector3d accl_bias_,
    const Eigen::Vector3d gyro_bias_
)
{
    tool.write_yaml(
        accl_bias_,
        save_bias_path,
        "accl_bias"
    );

    tool.write_yaml(
        gyro_bias_,
        save_bias_path,
        "gyro_bias"
    );

    GREEN_STREAM("SAVE BIAS TO YAML!");
}