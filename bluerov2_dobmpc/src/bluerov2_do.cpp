#include <bluerov2_dobmpc/bluerov2_do.h>

BLUEROV2_DO::BLUEROV2_DO(ros::NodeHandle& nh)
{
    nh.getParam("bluerov2_do_node/yaml_name",YAML_NAME);
    yaml_path = package_path + YAML_NAME;
    pose_gt_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/pose_gt", 20, &BLUEROV2_DO::pose_gt_cb, this);
    ref_pose_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/mpc/reference", 20, &BLUEROV2_DO::ref_pose_cb, this);
    control_input0_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/0", 20, &BLUEROV2_DO::control_input0_cb, this);
    control_input1_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/1", 20, &BLUEROV2_DO::control_input1_cb, this);
    control_input2_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/2", 20, &BLUEROV2_DO::control_input2_cb, this);
    control_input3_sub = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/3", 20, &BLUEROV2_DO::control_input3_cb, this);
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

void BLUEROV2_DO::control_input0_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    control_input0.data = msg->data;
}

void BLUEROV2_DO::control_input1_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    control_input1.data = msg->data;
}

void BLUEROV2_DO::control_input2_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    control_input2.data = msg->data;
}

void BLUEROV2_DO::control_input3_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    control_input3.data = msg->data;
}


// Define EKF function
void BLUEROV2_DO::EKF(VectorXd& x, MatrixXd& P, double u, VectorXd y, MatrixXd Q, MatrixXd R)
{
    // Define state and input dimensions
    int n = x.size(); // state dimension
    int m = y.size(); // measurement dimension

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
    F = compute_jacobian_F(x, u); // compute Jacobian of system dynamics at current state and input
    x_pred = f(x, u); // predict state at time k+1|k
    P_pred = F * P * F.transpose() + Q; // predict covariance at time k+1|k

    // Update step: correct state and covariance using measurement at time k+1
    H = compute_jacobian_H(x_pred); // compute Jacobian of measurement model at predicted state
    y_pred = h(x_pred); // predict measurement at time k+1
    y_err = y - y_pred; // compute measurement error
    K = P_pred * H.transpose() * (H * P_pred * H.transpose() + R).inverse(); // compute Kalman gain
    x = x_pred + K * y_err; // correct state estimate
    P = (MatrixXd::Identity(n, n) - K * H) * P_pred; // correct covariance estimate
}

// Define system dynamics function
VectorXd BLUEROV2_DO::f(VectorXd x, double u)
{
    // Define system dynamics
    VectorXd xdot(x.size());
    xdot << x(1), -x(0) + x(1) * u;
    return x + xdot * dt; // dt is the time step
}

// Define measurement model function
VectorXd BLUEROV2_DO::h(VectorXd x)
{
    // Define measurement model
    VectorXd y(1);
    y << x(0);
    return y;
}

// Define function to compute Jacobian of system dynamics at current state and input
MatrixXd BLUEROV2_DO::compute_jacobian_F(VectorXd x, double u)
{
    // Define Jacobian of system dynamics
    MatrixXd F(x.size(), x.size());
    F << 1, dt,
         -dt, 1 - dt * u;
    return F;
}

// Define function to compute Jacobian of measurement model at predicted state
MatrixXd BLUEROV2_DO::compute_jacobian_H(VectorXd x)
{
    // Define Jacobian of measurement model
    MatrixXd H(1, x.size());
    H << 1, 0;
    return H;
}

int main()
{
    // Define system parameters
    double u = 1.0; // input
    double dt = 0.1; // time step
    int n = 2; // state dimension
    int m = 1; // measurement dimension

    // Define initial state and covariance
    VectorXd x0(n);
    x0 << 0.0, 0.0; // initial state
    MatrixXd P0(n, n);
    P0 << 1.0, 0.0,
          0.0, 1.0; // initial covariance

    // Define process noise and measurement noise covariances
    MatrixXd Q(n, n);
    Q << 0.1, 0.0,
         0.0, 0.1; // process noise covariance
    MatrixXd R(m, m);
    R << 1.0; // measurement noise covariance

    // Define time vector and measurement vector
    int N = 100; // number of time steps
    VectorXd t(N); // time vector
    VectorXd y(N); // measurement vector
    for (int i = 0; i < N; i++) {
        t(i) = i * dt;
        y(i) = sin(t(i)); // measurement is sin(t)
    }

    // Run EKF algorithm
    VectorXd x = x0; // initialize state estimate
    MatrixXd P = P0; // initialize covariance estimate
    for (int i = 0; i < N; i++) {
        EKF(x, P, u, VectorXd::Constant(1, y(i)), Q, R); // run EKF algorithm
        cout << "t = " << t(i) << ", x = " << x.transpose() << endl; // print state estimate
    }

    return 0;
}

/*
This function takes in the system parameters, state vector, control input, operating point (ref), sampling time, 
and returns the discrete-time state-space model matrices. Note that this code assumes that the system has 6 measured states and 4 control inputs, 
so you may need to adjust the size of the matrices accordingly for your specific system.
x[k+1] = Ad * x[k] + Bd * u[k] + Gd 
y[k] = Cd * x[k] 


void BLUEROV2_DO::c2d_euler() 
{                           
    // Define the system model
    
    Cv(0,0) = -m*local_pos.r*local_pos.v + m*local_pos.q*local_pos.w;
    Cv(1,0) = m*local_pos.r*local_pos.u - m*local_pos.p*local_pos.w;
    Cv(2,0) = -m*local_pos.q*local_pos.u + m*local_pos.p*local_pos.v;
    Cv(3,0) = (Iz-Iy)*local_pos.q*local_pos.r;
    Cv(4,0) = (Ix-Iz)*local_pos.p*local_pos.r;
    Cv(5,0) = (Iy-Ix)*local_pos.p*local_pos.q;
    
    t(0,0) = -control_input0.data+control_input1.data+control_input3.data;
    t(1,0) = -control_input0.data-control_input1.data-control_input3.data;
    t(2,0) = control_input0.data+control_input1.data-control_input3.data;
    t(3,0) = control_input0.data-control_input1.data+control_input3.data;
    t(4,0) = -control_input2.data;
    t(5,0) = -control_input2.data;
    g(0,0) = bouyancy*sin(local_euler.theta);
    g(1,0) = -bouyancy*cos(local_euler.theta)*sin(local_euler.phi);
    g(2,0) = -bouyancy*cos(local_euler.theta)*cos(local_euler.phi);
    g(3,0) = m*ZG*g*cos(local_euler.theta)*sin(local_euler.phi);
    g(4,0) = m*ZG*g*sin(local_euler.theta);
    g(5,0) = 0;
    // derivation of state vector using the continous-time system model
    VectorXd f = M.inverse() * (K * t - Cv - g);

    // Linearize the system around the operating point
    MatrixXd A(12, 12);
    A.setZero();
    MatrixXd B(12, 4);
    B.setZero();
    VectorXd G(12);
    G.setZero();

    VectorXd x(12); // measured state vector
    x << local_pos.x,local_pos.y,local_pos.z,local_euler.phi,local_euler.theta,local_euler.psi,local_pos.u,local_pos.v,local_pos.w,local_pos.p,local_pos.q,local_pos.r;
    VectorXd x0(12); // operating point
    x0 << ref_pos.x,ref_pos.y,ref_pos.z,ref_euler.phi,ref_euler.theta,ref_euler.psi,ref_pos.u,ref_pos.v,ref_pos.w,ref_pos.p,ref_pos.q,ref_pos.r;
    VectorXd u0(12); //operating point
    u0 << control_input0.data,control_input1.data,control_input2.data,control_input3.data;

    for (int i = 0; i < 12; i++) {
        VectorXd x_temp = x;
        x_temp(i) = x0(i);
        A.col(i) = (f - M.inverse() * (K * u - C * x_temp.segment(6, 6) - g)) / (x(i) - x0(i));
    }
    for (int i = 0; i < 4; i++) {
        VectorXd u_temp = u;
        u_temp(i) = u0(i);
        B.col(i) = (f - M.inverse() * (K * u_temp - C * x.segment(6, 6) - g)) / (u(i) - u0(i));
    }
    G = f - A * x0 - B * u0;

    // Discretize the continuous-time model using the Euler integration method
    Ad = (MatrixXd::Identity(12, 12) + A * dt);
    Bd = B * dt;
    Gd = G * dt;

    // Define the measurement equation
    Cd = MatrixXd::Identity(6, 12);
}
*/
/*
In this function, the inputs are: 
-  z : the measurement vector, which includes all 12 states of x. 
-  A : the state transition matrix, which is a function of the system model. 
-  B : the input matrix, which is a function of the control allocation matrix. 
-  C : the measurement matrix, which maps the state vector to the measurement vector. 
-  Q : the process noise covariance matrix, which represents the uncertainty in the system model. 
-  R : the measurement noise covariance matrix, which represents the uncertainty in the measurements. 
-  x0 : the initial state estimate. 
-  P0 : the initial covariance estimate. 
-  u : the control input, which is a 4*1 matrix corresponds to surge, sway heave and yaw. 
The output of the function is the estimate of the state vector, which includes the state variables of the system and the disturbance term.

VectorXd BLUEROV2_DO::KalmanFilter(VectorXd z, MatrixXd Q, MatrixXd R, VectorXd x0, MatrixXd P0, VectorXd u)
{
    // Initialize variables
    int n = x0.size(); // number of states
    int m = z.size(); // number of measurements
    VectorXd x = x0; // initial state estimate
    MatrixXd P = P0; // initial covariance estimate
    MatrixXd I = MatrixXd::Identity(n, n); // identity matrix
    // Kalman filter loop
    for (int i = 0; i < m; i++) {
        // Prediction step
        x = Ad * x + Bd * u; // state prediction
        P = Ad * P * Ad.transpose() + Q; // covariance prediction
        // Update step
        VectorXd y = z - Cd * x; // measurement residual
        MatrixXd S = Cd * P * Cd.transpose() + R; // innovation covariance
        MatrixXd K = P * Cd.transpose() * S.inverse(); // Kalman gain
        x = x + K * y; // state update
        P = (I - K * Cd) * P; // covariance update
    }
    return x;
}
*/

/*
In this function, the inputs are: 
-  X : the design matrix, which includes the estimates of the state vector from the Kalman filter, as well as the control inputs. 
-  y : the response vector, which is the measurements of the system's response to external inputs. 
-  theta0 : the initial parameter estimate. 
-  lambda : the forgetting factor, which determines the rate at which past measurements are forgotten. 
The output of the function is the estimate of the disturbance term. 
VectorXd RecursiveLeastSquares(MatrixXd X, VectorXd y, VectorXd theta0, double lambda)
{
    // Initialize variables
    int m = X.rows(); // number of samples
    int n = X.cols(); // number of parameters
    VectorXd theta = theta0; // initial parameter estimate
    MatrixXd P = lambda * MatrixXd::Identity(n, n); // initial covariance estimate
    // Recursive least squares loop
    for (int i = 0; i < m; i++) {
        // Update step
        double e = y(i) - X.row(i) * theta; // prediction error
        double k = (P * X.row(i).transpose()) / (1 + X.row(i) * P * X.row(i).transpose()); // Kalman gain
        theta = theta + k * e; // parameter update
        P = (P - k * X.row(i) * P) / lambda; // covariance update
    }
    return theta;
}
*/


/*
Kalman filter: 
- Prediction step: 
  - x(k|k-1) = A * x(k-1|k-1) + B * u(k-1) 
  - P(k|k-1) = A * P(k-1|k-1) * A^T + Q 
- Update step: 
  - y(k) = z(k) - C * x(k|k-1) 
  - S(k) = C * P(k|k-1) * C^T + R 
  - K(k) = P(k|k-1) * C^T * S(k)^(-1) 
  - x(k|k) = x(k|k-1) + K(k) * y(k) 
  - P(k|k) = (I - K(k) * C) * P(k|k-1) 
 
Recursive least squares: 
- Update step: 
  - e(k) = y(k) - X(k) * theta(k-1) 
  - k(k) = P(k-1) * X(k)^T * (X(k) * P(k-1) * X(k)^T + lambda)^(-1) 
  - theta(k) = theta(k-1) + k(k) * e(k) 
  - P(k) = (P(k-1) - k(k) * X(k) * P(k-1)) / lambda 
 
In these formulas, the variables are defined as follows: 
-  x : the state vector, which includes the state variables of the system and the disturbance term. 
-  u : the control input. 
-  z : the measurement vector, which includes all 12 states of x. 
-  A : the state transition matrix. 
-  B : the input matrix. 
-  C : the measurement matrix. 
-  Q : the process noise covariance matrix. 
-  R : the measurement noise covariance matrix. 
-  P : the covariance matrix. 
-  K : the Kalman gain. 
-  y : the response vector, which is the measurements of the system's response to external inputs. 
-  X : the design matrix, which includes the estimates of the state vector from the Kalman filter, as well as the control inputs. 
-  theta : the parameter vector, which represents the disturbance term. 
-  lambda : the forgetting factor, which determines the rate at which past measurements are forgotten.
*/