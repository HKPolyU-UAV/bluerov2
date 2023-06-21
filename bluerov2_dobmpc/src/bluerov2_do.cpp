#include <bluerov2_dobmpc/bluerov2_do.h>

BLUEROV2_DO::BLUEROV2_DO(ros::NodeHandle& nh)
{
    nh.getParam("bluerov2_do_node/yaml_name",YAML_NAME);
    yaml_path = package_path + YAML_NAME;
    pose_gt_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/pose_gt", 20, &BLUEROV2_DO::pose_gt_cb, this);
    ref_pose_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/mpc/reference", 20, &BLUEROV2_DO::ref_pose_cb, this);
    thrust0_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/0/input", 20, &BLUEROV2_DO::thrust0_cb, this);
    thrust1_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/1/input", 20, &BLUEROV2_DO::thrust1_cb, this);
    thrust2_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/2/input", 20, &BLUEROV2_DO::thrust2_cb, this);
    thrust3_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/3/input", 20, &BLUEROV2_DO::thrust3_cb, this);
    thrust2_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/4/input", 20, &BLUEROV2_DO::thrust4_cb, this);
    thrust3_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/5/input", 20, &BLUEROV2_DO::thrust5_cb, this);

    // Initialize estimate state and covariance
    esti_x = x0;
    esti_P = P0;
}

void BLUEROV2_DO::pose_gt_cb(const nav_msgs::Odometry::ConstPtr& pose)
{
    // get linear position x, y, z
    local_pos.x = pose->pose.pose.position.x;
    local_pos.y = pose->pose.pose.position.y;
    local_pos.z = pose->pose.pose.position.z;

    // get angle phi, theta, psi
    tf::quaternionMsgToTF(pose->pose.pose.orientation,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(local_euler.phi, local_euler.theta, local_euler.psi);

    // get linear velocity u, v, w
    local_pos.u = pose->twist.twist.linear.x;
    local_pos.v = pose->twist.twist.linear.y;
    local_pos.w = pose->twist.twist.linear.z;

    // get angular velocity p, q, r
    local_pos.p = pose->twist.twist.angular.x;
    local_pos.q = pose->twist.twist.angular.y;
    local_pos.r = pose->twist.twist.angular.z;
}

void BLUEROV2_DO::ref_pose_cb(const nav_msgs::Odometry::ConstPtr& pose)
{
    // get linear position x, y, z
    ref_pos.x = pose->pose.pose.position.x;
    ref_pos.y = pose->pose.pose.position.y;
    ref_pos.z = pose->pose.pose.position.z;

    // get angle phi, theta, psi
    tf::quaternionMsgToTF(pose->pose.pose.orientation,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(ref_euler.phi, ref_euler.theta, ref_euler.psi);

    // get linear velocity u, v, w
    ref_pos.u = pose->twist.twist.linear.x;
    ref_pos.v = pose->twist.twist.linear.y;
    ref_pos.w = pose->twist.twist.linear.z;                                 

    // get angular velocity p, q, r
    ref_pos.p = pose->twist.twist.angular.x;
    ref_pos.q = pose->twist.twist.angular.y;
    ref_pos.r = pose->twist.twist.angular.z;
}

void BLUEROV2_DO::thrust0_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    thrust0.data = msg->data;
}

void BLUEROV2_DO::thrust1_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    thrust1.data = msg->data;
}

void BLUEROV2_DO::thrust2_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    thrust2.data = msg->data;
}

void BLUEROV2_DO::thrust3_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    thrust3.data = msg->data;
}

void BLUEROV2_DO::thrust4_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    thrust4.data = msg->data;
}

void BLUEROV2_DO::thrust5_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    thrust5.data = msg->data;
}
// Define EKF function
// inputs: current state estimate x, current covariance estimate P, input u, measurement y, 
//        process noise covariance Q, measuremnt noise covariance R
void BLUEROV2_DO::EKF()
{
    // get input u and measuremnet y
    u << thrust0.data, thrust1.data, thrust2.data, thrust3.data, thrust4.data, thrust5.data;
    meas_y << ref_pos.x, ref_pos.y, ref_pos.z, ref_euler.phi, ref_euler.theta, ref_euler.psi;

    // Define Jacobian matrices of system dynamics and measurement model
    MatrixXd F(n, n); // Jacobian of system dynamics
    MatrixXd H(m, n); // Jacobian of measurement model

    // Define Kalman gain matrix
    MatrixXd K(n, m);

    // Define prediction and update steps
    VectorXd x_pred(n); // predicted state
    MatrixXd P_pred(n, n); // predicted covariance
    VectorXd y_pred(m); // predicted measurement
    VectorXd y_err(m); // measurement error

    // Prediction step: estimate state and covariance at time k+1|k
    F = compute_jacobian_F(esti_x, u); // compute Jacobian of system dynamics at current state and input
    x_pred = f(esti_x, u); // predict state at time k+1|k
    P_pred = F * esti_P * F.transpose() + noise_Q; // predict covariance at time k+1|k

    // Update step: correct state and covariance using measurement at time k+1
    H = compute_jacobian_H(x_pred); // compute Jacobian of measurement model at predicted state
    y_pred = h(x_pred); // predict measurement at time k+1
    y_err = meas_y - y_pred; // compute measurement error
    K = P_pred * H.transpose() * (H * P_pred * H.transpose() + noise_R).inverse(); // compute Kalman gain
    esti_x = x_pred + K * y_err; // correct state estimate
    esti_P = (MatrixXd::Identity(n, n) - K * H) * P_pred; // correct covariance estimate
}

// Define system dynamics function
VectorXd BLUEROV2_DO::f(VectorXd x, VectorXd u)
{
    // Define system dynamics
    VectorXd xdot(n);

    KAu = K*u.transpose();
    xdot << x(6),x(7),x(8),x(9),(10),(11),                          // xdot
            invM.coeff(0)(0)*(KAu(0)+x(12)+m*x(11)*x(7)-m*x(10)*x(8)-bouyancy*sin(x(4))),    // M^-1[tau+w-C-g]
            invM.coeff(1)(1)*(KAu(1)+x(13)-m*x(11)*x(6)+m*x(9)*x(8)+bouyancy*cos(x(4))*sin(x(3))),
            invM.coeff(2)(2)*(KAu(2)+x(14)+m*x(10)*x(6)-m*x(9)*x(7)+bouyancy*cos(x(4))*cos(x(3))),
            invM.coeff(3)(3)*(KAu(3)+x(15)+(Iy-Iz)*x(10)*x(11)-m*ZG*g*cos(x(4))*sin(x(3))),
            invM.coeff(4)(4)*(KAu(4)+x(16)+(Iz-Ix)*x(9)*x(11)-m*ZG*g*sin(x(4))),
            invM.coeff(5)(5)*(KAu(5)+x(17)-(Iy-Ix)*x(9)*x(10)),
            
        
    return x + xdot * dt; // dt is the time step
}

// Define measurement model function
VectorXd BLUEROV2_DO::h(VectorXd x)
{
    // Define measurement model
    VectorXd y(m);
    


    return y;
}

// Define function to compute Jacobian of system dynamics at current state and input
MatrixXd BLUEROV2_DO::compute_jacobian_F(VectorXd x, double u)
{
    // Define Jacobian of system dynamics
    MatrixXd F(x.size(), x.size());
    
    return F;
}

// Define function to compute Jacobian of measurement model at predicted state
MatrixXd BLUEROV2_DO::compute_jacobian_H(VectorXd x)
{
    // Define Jacobian of measurement model
    MatrixXd H(1, x.size());
    
    return H;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluerov2_do_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    // Run EKF algorithm
    BLUEROV2_DO do(nh);
    while(ros::ok()){
        do.EKF(); // run EKF algorithm
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

