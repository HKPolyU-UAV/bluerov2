#include "bluerov2_states/ImuDo.h"

void BLUEROV2_STATES::ImuDoNodelet::communi_config(ros::NodeHandle& nh)
{
    gt_sub = nh.subscribe<nav_msgs::Odometry>
                ("/bluerov2/pose_gt", 1, &ImuDoNodelet::pose_gt_callback, this);
    
    imu_sub = nh.subscribe<sensor_msgs::Imu>
                ("/bluerov2/imu", 1, &ImuDoNodelet::imu_callback, this);
    
    gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
                ("/bluerov2/gps", 1, &ImuDoNodelet::gps_callback, this);

    for (int i = 0; i < 6; i++)
    {
        std::string topic = "/bluerov2/thrusters/" + std::to_string(i) + "/thrust";
        th_subs.emplace_back(
            nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>(
                topic, 
                20, 
                boost::bind(&ImuDoNodelet::thrusts_cb, this, _1, i)
            )
        );
    }

    main_spin_timer = nh.createTimer(
        ros::Duration(0.01), 
        &ImuDoNodelet::main_spin_callback, 
        this
    );

    update_pub = nh.advertise<std_msgs::Bool>
                ("/dummy_update", 1);
    imu_pub = nh.advertise<std_msgs::Bool>
                ("/dummy_imu", 1);

    esti_dist_pub = nh.advertise<airo_message::Disturbance>
                ("/disturbance", 1);

    xi_pub = nh.advertise<geometry_msgs::Point>
                ("/xi", 1);

    est_pub = nh.advertise<nav_msgs::Odometry>
                ("/est_x", 1);

    wrench_pub = nh.advertise<geometry_msgs::Point>("/current_wrench",1);

    ROS_CYAN_STREAM("COMMUNICATION CONFIG SUCCEEDED!");
}

void BLUEROV2_STATES::ImuDoNodelet::eskf_config(ros::NodeHandle& nh)
{
    using namespace std;

    init_odom(nh);
    init_bias(nh);
    init_disturb(nh);
    init_noise(nh);
    init_gps(nh);

    t_prev = ros::Time::now().toSec();
    
    ROS_CYAN_STREAM("ESKF CONFIG SUCCEEDED!");
}

void BLUEROV2_STATES::ImuDoNodelet::init_odom(ros::NodeHandle& nh)
{
    std::cout<<"gan"<<std::endl;
    // initialize state values
    while(ros::ok())
    {
        if(
            got_gps && 
            got_imu &&
            got_gt
        )
            break;

        std::cout<<"huh"<<std::endl;

        ros::spinOnce();
        ros::Rate(10).sleep();
    }

    SE3_est_I = vehicle_SE3_world_gt;
    v_est_I = vehicle_twist_world_gt.head(3);

    inertial_start = SE3_est_I.translation();

    g_vector_est_I.z() = - g_constant;
}

void BLUEROV2_STATES::ImuDoNodelet::init_bias(ros::NodeHandle& nh)
{
    XmlRpc::XmlRpcValue accl_bias_load_list;
    XmlRpc::XmlRpcValue gyro_bias_load_list;

    nh.getParam("/BLUEROV2_STATES_master/accl_bias", accl_bias_load_list);
    nh.getParam("/BLUEROV2_STATES_master/gyro_bias", gyro_bias_load_list);

    std::ostringstream ostr;
    std::istringstream istr;

    for (int i = 0; i < 3; i++) {
        b_a_est_B(i) = static_cast<double>(accl_bias_load_list[i]);
        b_g_est_B(i) = static_cast<double>(gyro_bias_load_list[i]);
    }

    // std::cout<<b_a_est_B<<std::endl;
    // std::cout<<b_g_est_B<<std::endl;
}

void BLUEROV2_STATES::ImuDoNodelet::init_disturb(ros::NodeHandle& nh)
{
    xi_est_I.setZero();
}

void BLUEROV2_STATES::ImuDoNodelet::init_noise(ros::NodeHandle& nh)
{
    double p_proc_noise, v_proc_noise, r_proc_noise, xi_proc_noise, q_proc_noise;

    nh.getParam("/BLUEROV2_STATES_master/p_proc_noise", p_proc_noise);
    nh.getParam("/BLUEROV2_STATES_master/v_proc_noise", v_proc_noise);
    nh.getParam("/BLUEROV2_STATES_master/r_proc_noise", r_proc_noise);
    nh.getParam("/BLUEROV2_STATES_master/xi_proc_noise", xi_proc_noise);
    nh.getParam("/BLUEROV2_STATES_master/q_proc_noise", q_proc_noise);
    
    Q_process = (
        Eigen::Matrix<double, 21, 1>()
        <<
        Eigen::Vector3d::Ones() * p_proc_noise,
        Eigen::Vector3d::Ones() * v_proc_noise,
        Eigen::Vector3d::Ones() * r_proc_noise,
        Eigen::Vector3d::Ones() * q_proc_noise,
        Eigen::Vector3d::Ones() * q_proc_noise,
        Eigen::Vector3d::Ones() * q_proc_noise,
        Eigen::Vector3d::Ones() * xi_proc_noise
    ).finished().asDiagonal();

    double p_meas_noise, v_meas_noise, r_meas_noise, th_meas_noise;
    
    nh.getParam("/BLUEROV2_STATES_master/p_meas_noise", p_meas_noise);
    nh.getParam("/BLUEROV2_STATES_master/v_meas_noise", v_meas_noise);
    nh.getParam("/BLUEROV2_STATES_master/r_meas_noise", r_meas_noise);
    nh.getParam("/BLUEROV2_STATES_master/th_meas_noise", th_meas_noise);

    R_meas = (
        Eigen::Matrix<double, 12, 1>() 
        << 
        Eigen::Vector3d::Ones() * p_meas_noise, 
        Eigen::Vector3d::Ones() * v_meas_noise, 
        Eigen::Vector3d::Ones() * r_meas_noise, 
        Eigen::Vector3d::Ones() * th_meas_noise
    ).finished().asDiagonal();
}

void BLUEROV2_STATES::ImuDoNodelet::init_gps(ros::NodeHandle& nh)
{
    gps_start = Eigen::Vector3d(
        gps_buf.back()->longitude,
        gps_buf.back()->latitude,
        gps_buf.back()->altitude
    );
    gps_buf.pop();

    curr_gps_trans_I.setZero();
    prev_gps_trans_I.setZero();
    gps_t_prev = ros::Time::now().toSec();
}

void BLUEROV2_STATES::ImuDoNodelet::dynamics_parameter_config(
    ros::NodeHandle& nh
)
{
    K << 0.7071067811847433, 0.7071067811847433, -0.7071067811919605, -0.7071067811919605, 0.0, 0.0,
       0.7071067811883519, -0.7071067811883519, 0.7071067811811348, -0.7071067811811348, 0.0, 0.0,
       0, 0, 0, 0, 1, 1,
       0.051265241636155506, -0.05126524163615552, 0.05126524163563227, -0.05126524163563227, -0.11050000000000001, 0.11050000000000003,
       -0.05126524163589389, -0.051265241635893896, 0.05126524163641713, 0.05126524163641713, -0.002499999999974481, -0.002499999999974481,
       0.16652364696949604, -0.16652364696949604, -0.17500892834341342, 0.17500892834341342, 0.0, 0.0;
    
    M_rb = (
        Sophus::Vector6d()
        <<
        mass, mass, mass, Ix, Iy, Iz
    ).finished().asDiagonal();
    
    M = M_values.asDiagonal();
    M_rb(0,4) = mass * ZG;
    M_rb(1,3) = - mass * ZG;
    M_rb(3,1) = - mass * ZG;
    M_rb(4,0) = mass * ZG;

    M_a = (
        Sophus::Vector6d()
        <<
        added_mass[0],
        added_mass[1],
        added_mass[2],
        added_mass[3],
        added_mass[4],
        added_mass[5]
    ).finished().asDiagonal();

    M = M_rb + M_a;
    invM = M.inverse();

    ROS_CYAN_STREAM("DYNAMIC CONFIG SUCCEEDED!");
}