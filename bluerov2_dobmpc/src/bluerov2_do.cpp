#include <bluerov2_dobmpc/bluerov2_do.h>

BLUEROV2_DO::BLUEROV2_DO(ros::NodeHandle& nh)
{
    nh.getParam("bluerov2_do_node/yaml_name",YAML_NAME);
    yaml_path = package_path + YAML_NAME;
    pose_gt_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/pose_gt", 20, &BLUEROV2_DO::pose_gt_cb, this);
    thrust0_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/0/input", 20, &BLUEROV2_DO::thrust0_cb, this);
    thrust1_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/1/input", 20, &BLUEROV2_DO::thrust1_cb, this);
    thrust2_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/2/input", 20, &BLUEROV2_DO::thrust2_cb, this);
    thrust3_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/3/input", 20, &BLUEROV2_DO::thrust3_cb, this);
    thrust2_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/4/input", 20, &BLUEROV2_DO::thrust4_cb, this);
    thrust3_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/5/input", 20, &BLUEROV2_DO::thrust5_cb, this);
    esti_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/ekf/pose",20);

    // Initialize system parameters
    M_values << mass + added_mass[0], mass + added_mass[1], mass + added_mass[2], Ix + added_mass[3], Iy + added_mass[4], Iz + added_mass[5];
    M = M_values.asDiagonal();
    invM = M.inverse();
    K << 0.707, 0.707, -0.707, -0.707, 0, 0,
       0.707, -0.707, 0.707, -0.707, 0, 0,
       0, 0, 0, 0, 1, 1,
       0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0,
       0.167, -0.167, -0.175, 0.175, 0, 0;
       
    Q_cov << pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,
            pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),
            1,1,1,1,1,1;
    noise_Q= Q_cov.asDiagonal();
    
    // Initialize estimate state and covariance
    x0.fill(0);
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
    meas_y << local_pos.x, local_pos.y, local_pos.z, local_euler.phi, local_euler.theta, local_euler.psi,
            local_pos.u, local_pos.v, local_pos.w, local_pos.p, local_pos.q, local_pos.r,
            thrust0.data, thrust1.data, thrust2.data, thrust3.data, thrust4.data, thrust5.data;
    
    // Define Jacobian matrices of system dynamics and measurement model
    Matrix<double,18,18> F; // Jacobian of system dynamics
    Matrix<double,18,18> H; // Jacobian of measurement model

    // Define Kalman gain matrix
    Matrix<double,18,18> Kal;

    // Define prediction and update steps
    Matrix<double,18,1> x_pred; // predicted state
    Matrix<double,18,18> P_pred; // predicted covariance
    Matrix<double,18,1> y_pred; // predicted measurement
    Matrix<double,18,1> y_err; // measurement error
    
    // Prediction step: estimate state and covariance at time k+1|k
    F = compute_jacobian_F(esti_x, u); // compute Jacobian of system dynamics at current state and input
    x_pred = f(esti_x, u); // predict state at time k+1|k
    P_pred = F * esti_P * F.transpose() + noise_Q; // predict covariance at time k+1|k
    
    // Update step: correct state and covariance using measurement at time k+1
    H = compute_jacobian_H(x_pred); // compute Jacobian of measurement model at predicted state
    y_pred = h(x_pred); // predict measurement at time k+1
    y_err = meas_y - y_pred; // compute measurement error
    Kal = P_pred * H.transpose() * (H * P_pred * H.transpose() + noise_R).inverse(); // compute Kalman gain
    esti_x = x_pred + Kal * y_err; // correct state estimate
    esti_P = (MatrixXd::Identity(n, n) - Kal * H) * P_pred; // correct covariance estimate
    
    // publish estimate pose
    tf2::Quaternion quat;
    quat.setRPY(esti_x(3), esti_x(4), esti_x(5));
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat, quat_msg);
    esti_pose.pose.pose.position.x = esti_x(0);
    esti_pose.pose.pose.position.y = esti_x(1);
    esti_pose.pose.pose.position.z = esti_x(2);
    esti_pose.pose.pose.orientation.x = quat_msg.x;
    esti_pose.pose.pose.orientation.y = quat_msg.y;
    esti_pose.pose.pose.orientation.z = quat_msg.z;
    esti_pose.pose.pose.orientation.w = quat_msg.w;
    esti_pose.twist.twist.linear.x = esti_x(6);
    esti_pose.twist.twist.linear.y = esti_x(7);
    esti_pose.twist.twist.linear.z = esti_x(8);
    esti_pose.twist.twist.angular.x = esti_x(9);
    esti_pose.twist.twist.angular.y = esti_x(10);
    esti_pose.twist.twist.angular.z = esti_x(11);
    esti_pose.header.stamp = ros::Time::now();
    esti_pose.header.frame_id = "odom_frame";
    esti_pose.child_frame_id = "base_link";
    esti_pose_pub.publish(esti_pose);

    // print estimate disturbance
    if(cout_counter > 2){
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        std::cout << "thrust0: " << u(0) << "  thrust1: " << u(1) << "  thrust2: " << u(2) << "  thrust3: " << u(3) << "  thrust4: " << u(4) << "  thrust5: " << u(5) <<std::endl; 
        std::cout << "pos_x: " << meas_y(0) << "  pos_y: " << meas_y(1) << "  pos_z: " << meas_y(2) << " phi: " << meas_y(3) << "  theta: " << meas_y(4) << "  psi: " << meas_y(5) <<std::endl;
        std::cout << "esti_x: " << esti_x(0) << "  esti_y: " << esti_x(1) << "  esti_z: " << esti_x(2) << " esti_phi: " << esti_x(3) << "  esti_theta: " << esti_x(4) << "  esti_psi: " << esti_x(5) <<std::endl;
        std::cout << "pos_u: " << meas_y(6) << "  pos_v: " << meas_y(7) << "  pos_w: " << meas_y(8) << " pos_p: " << meas_y(9) << "  pos_q: " << meas_y(10) << "  pos_r: " << meas_y(11) <<std::endl;
        std::cout << "esti_u: " << esti_x(6) << "  esti_v: " << esti_x(7) << "  esti_w: " << esti_x(8) << " esti_p: " << esti_x(9) << "  esti_q: " << esti_x(10) << "  esti_r: " << esti_x(11) <<std::endl;
        std::cout << "disturbance x: " << esti_x(12) << "    disturbance y: " << esti_x(13) << "    disturbance z: " << esti_x(14) << std::endl;
        std::cout << "disturbance phi: " << esti_x(15) << "    disturbance theta: " << esti_x(16) << "    disturbance psi: " << esti_x(17) << std::endl;
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        cout_counter = 0;
    }
    else{
        cout_counter++;
    }
}

// Define system dynamics function
MatrixXd BLUEROV2_DO::f(MatrixXd x, MatrixXd u)
{
    // Define system dynamics
    Matrix<double,18,1> xdot;

    KAu = K*u;
    xdot << (cos(x(5))*cos(x(4)))*x(6) + (-sin(x(5))*cos(x(3))+cos(x(5))*sin(x(4))*sin(x(3)))*x(7) + (sin(x(5))*sin(x(3))+cos(x(5))*cos(x(3))*sin(x(4)))*x(8),  //xdot
            (sin(x(5))*cos(x(4)))*x(6) + (cos(x(5))*cos(x(3))+sin(x(3))*sin(x(4))*sin(x(5)))*x(7) + (-cos(x(5))*sin(x(3))+sin(x(4))*sin(x(5))*cos(x(3)))*x(8),
            (-sin(x(4)))*x(6) + (cos(x(4))*sin(x(3)))*x(7) + (cos(x(4))*cos(x(3)))*x(8),
            x(9) + (sin(x(5))*sin(x(4))/cos(x(4)))*x(10) + cos(x(3))*sin(x(4))/cos(x(4))*x(11),
            (cos(x(3)))*x(10) + (sin(x(3)))*x(11),
            (sin(x(3))/cos(x(4)))*x(10) + (cos(x(3))/cos(x(4)))*x(11),
            invM(0,0)*(KAu(0)+x(12)+mass*x(11)*x(7)-mass*x(10)*x(8)-bouyancy*sin(x(4))),    // xddot: M^-1[tau+w-C-g]
            invM(1,1)*(KAu(1)+x(13)-mass*x(11)*x(6)+mass*x(9)*x(8)+bouyancy*cos(x(4))*sin(x(3))),
            invM(2,2)*(KAu(2)+x(14)+mass*x(10)*x(6)-mass*x(9)*x(7)+bouyancy*cos(x(4))*cos(x(3))),
            invM(3,3)*(KAu(3)+x(15)+(Iy-Iz)*x(10)*x(11)-mass*ZG*g*cos(x(4))*sin(x(3))),
            invM(4,4)*(KAu(4)+x(16)+(Iz-Ix)*x(9)*x(11)-mass*ZG*g*sin(x(4))),
            invM(5,5)*(KAu(5)+x(17)-(Iy-Ix)*x(9)*x(10)),
            Cp(0,0)*invM(0,0)*x(12),Cp(1,1)*invM(1,1)*x(13),Cp(2,2)*invM(2,2)*x(14),   // wdot
            Cp(3,3)*invM(3,3)*x(15),Cp(4,4)*invM(4,4)*x(16),Cp(5,5)*invM(5,5)*x(17);   
            
    return x + xdot * dt; // dt is the time step
}

// Define measurement model function (Z = Hx, Z: measurement vector [x,xdot,tau]; X: state vector [x,xdot,disturbance])
MatrixXd BLUEROV2_DO::h(MatrixXd x)
{
    // Define measurement model
    Matrix<double,18,1> y;
    y << x(0),x(1),x(2),x(3),x(4),x(5),
        x(6),x(7),x(8),x(9),x(10),x(11),
        Cp(0,0)*x(6)-mass*x(11)*x(7)+mass*x(10)*x(8)+bouyancy*sin(x(4))-x(12),        // M*xddot = p = c*xdot
        Cp(1,1)*x(7)+mass*x(11)*x(6)-mass*x(9)*x(8)-bouyancy*cos(x(4))*sin(x(3))-x(13),
        Cp(2,2)*x(8)-mass*x(10)*x(6)+mass*x(9)*x(7)-bouyancy*cos(x(4))*cos(x(3))-x(14),
        Cp(3,3)*x(9)-(Iy-Iz)*x(10)*x(11)+mass*ZG*g*cos(x(4))*sin(x(3))-x(15),
        Cp(4,4)*x(10)-(Iz-Ix)*x(9)*x(11)+mass*ZG*g*sin(x(4))-x(16),
        Cp(5,5)*x(11)+(Iy-Ix)*x(9)*x(10)-x(17);

    return y;
}

// Define function to compute Jacobian of system dynamics at current state and input
MatrixXd BLUEROV2_DO::compute_jacobian_F(MatrixXd x, MatrixXd u)
{
    // Define Jacobian of system dynamics
    Matrix<double,18,18> F;
    double h = 1e-6;                    // finite difference step size
    VectorXd f0 = f(x, u);
    for (int i = 0; i < n; i++){
        VectorXd x1 = x;
        x1(i) += h;
        VectorXd f1 = f(x1, u);
        F.col(i) = (f1-f0)/h;
    }
    return F;
}

// Define function to compute Jacobian of measurement model at predicted state
MatrixXd BLUEROV2_DO::compute_jacobian_H(MatrixXd x)
{
    // Define Jacobian of measurement model
    Matrix<double,18,18> H;
    double d = 1e-6;                    // finite difference step size
    VectorXd f0 = h(x);
    for (int i = 0; i < n; i++){
        VectorXd x1 = x;
        x1(i) += d;
        VectorXd f1 = h(x1);
        H.col(i) = (f1-f0)/d;
    }

    return H;
}



