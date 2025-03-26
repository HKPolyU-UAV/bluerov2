#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import Imu, FluidPressure
from uuv_sensor_ros_plugins_msgs.msg import DVL
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class UnscentedKalmanFilterNode:
    def __init__(self):
        rospy.init_node('ukf_sensor_fusion', anonymous=True)
        
        # Initialize sensor data
        self.imu_data = None
        self.dvl_data = None
        self.pressure_data = None
        self.last_time = rospy.Time.now()

        # Initialize path and position
        self.path = Path()
        self.path.header.frame_id = "map"
        self.position_x = -12.0
        self.position_y = 0.0
        self.position_z = -95.0

        # UKF variables
        self.state = np.array([-12.0, 0.0, -95.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [x, y, z, roll, pitch, yaw, vx, vy, vz]
        # self.P = np.eye(9) * 0.1  # State covariance matrix
        # self.Q = np.diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])  # Process noise covariance
        self.R = np.diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1])  # Measurement noise covariance

        self.P = np.eye(9) * 10.0  
        self.Q = np.diag([0.1, 0.1, 0.1,  # 位置噪声
                  0.01, 0.01, 0.01,  # 姿态噪声
                  0.5, 0.5, 0.5])  # 速度噪声
        self.R[2,2] = 0.5  # 深度噪声

        # Subscribe to sensor topics
        rospy.Subscriber("/bluerov2/imu", Imu, self.imu_callback)
        rospy.Subscriber("/bluerov2/dvl", DVL, self.dvl_callback)
        rospy.Subscriber("/bluerov2/pressure", FluidPressure, self.pressure_callback)

        # Publish fused odometry and path
        self.odom_pub = rospy.Publisher("/bluerov2/fused_odom_ukf", Odometry, queue_size=10)
        self.path_pub = rospy.Publisher("/bluerov2/fused_trajectory_ukf", Path, queue_size=10)
        self.tf_broadcaster = TransformBroadcaster()

    def imu_callback(self, msg):
        self.imu_data = {
            'angular_vel': msg.angular_velocity,
            'linear_acc': msg.linear_acceleration,
            'orientation': msg.orientation,
            'timestamp': msg.header.stamp
        }
        
    def dvl_callback(self, msg):
        self.dvl_data = {
            'velocity': msg.velocity,
            'timestamp': msg.header.stamp
        }
        
    def pressure_callback(self, msg):
        water_density = 1000  # kg/m³
        gravity = 9.81
        self.pressure_data = {
            'depth': -(msg.fluid_pressure * 1000 - 101325) / (water_density * gravity),
            'timestamp': msg.header.stamp
        }

    def state_transition(self, state, dt):
        # State transition function
        roll, pitch, yaw = state[3:6]
        vx, vy, vz = state[6:9]

        next_state = state.copy()
        next_state[0] += vx * dt
        next_state[1] += vy * dt
        next_state[2] += vz * dt
        next_state[3:6] = self.update_orientation(state[3:6], state[6:9], dt)

        return next_state

    def update_orientation(self, rpy, angular_vel, dt):
        # Update orientation using angular velocity
        roll, pitch, yaw = rpy
        wx, wy, wz = angular_vel
        next_roll = roll + wx * dt
        next_pitch = pitch + wy * dt
        next_yaw = yaw + wz * dt
        return np.array([next_roll, next_pitch, next_yaw])

    def measurement_function(self, state):
        # Measurement function
        roll, pitch, yaw = state[3:6]
        vx_body = self.dvl_data['velocity'].x
        vy_body = self.dvl_data['velocity'].y
        vz_body = self.dvl_data['velocity'].z

        # Convert body frame velocity to world frame
        vx_world = vx_body * np.cos(yaw) - vy_body * np.sin(yaw)
        vy_world = vx_body * np.sin(yaw) + vy_body * np.cos(yaw)
        vz_world = vz_body

        # measurement = np.array([
        #     state[0], state[1], self.pressure_data['depth'],
        #     roll, pitch, yaw,
        #     vx_world, vy_world, vz_world
        # ])
        measurement = np.array([
            state[0], state[1], state[2],
            roll, pitch, yaw,
            vx_world, vy_world, vz_world
        ])
        return measurement

    def unscented_kalman_filter(self):
        if None in [self.imu_data, self.dvl_data, self.pressure_data]:
            return
            
        try:
            # Time synchronization (using IMU time)
            current_time = self.imu_data['timestamp']
            dt = (current_time - self.last_time).to_sec()
            if dt <= 0:
                return

            # Predict step
            self.state = self.state_transition(self.state, dt)
            self.P = self.predict_covariance(self.state, self.P, dt, self.Q)

            # Measurement update
            z = self.measurement_function(self.state)
            y = self.measurement_function(np.array([self.position_x, self.position_y, self.position_z, self.state[3], self.state[4], self.state[5], self.state[6], self.state[7], self.state[8]])) - z
            S = self.measurement_covariance(self.state, self.P, self.R)
            K = self.kalman_gain(self.P, self.measurement_jacobian(self.state), S)
            self.state = self.state + np.dot(K, y)
            self.P = self.P - np.dot(K, np.dot(self.measurement_jacobian(self.state), self.P))

            # Integrate position
            self.position_x = self.state[0]
            self.position_y = self.state[1]
            self.position_z = self.pressure_data['depth']

            # Create Odometry message
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.position_x
            odom.pose.pose.position.y = self.position_y
            odom.pose.pose.position.z = self.position_z
            odom.pose.pose.orientation = self.imu_data['orientation']
            odom.twist.twist.linear.x = self.state[6]
            odom.twist.twist.linear.y = self.state[7]
            odom.twist.twist.linear.z = self.state[8]
            odom.twist.twist.angular = self.imu_data['angular_vel']

            # Publish TF
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.position_x
            t.transform.translation.y = self.position_y
            t.transform.translation.z = self.position_z
            t.transform.rotation = self.imu_data['orientation']
            self.tf_broadcaster.sendTransform(t)

            # Update path
            pose_stamped = PoseStamped()
            pose_stamped.header = odom.header
            pose_stamped.pose = odom.pose.pose
            self.path.poses.append(pose_stamped)
            self.path_pub.publish(self.path)
            self.odom_pub.publish(odom)
            
            self.last_time = current_time
            
        except Exception as e:
            rospy.logerr("Fusion error: %s" % str(e))

    def predict_covariance(self, state, P, dt, Q):
        # Predict covariance using the Jacobian of the state transition function
        F = self.state_transition_jacobian(state, dt)
        return np.dot(np.dot(F, P), F.T) + Q

    def state_transition_jacobian(self, state, dt):
        # Compute the Jacobian of the state transition function
        roll, pitch, yaw = state[3:6]
        vx, vy, vz = state[6:9]
        
        J = np.eye(9)
        J[0, 6] = dt
        J[1, 7] = dt
        J[2, 8] = dt
        J[3:6, 6:9] = self.orientation_jacobian(state[3:6], state[6:9], dt)
        
        return J

    # def orientation_jacobian(self, rpy, angular_vel, dt):
    #     # Compute the Jacobian of the orientation update
    #     roll, pitch, yaw = rpy
    #     wx, wy, wz = angular_vel
        
    #     J = np.array([
    #         [dt, 0, 0],
    #         [0, dt, 0],
    #         [0, 0, dt]
    #     ])
        
    #     return J

    def orientation_jacobian(self, rpy, angular_vel, dt):
        wx, wy, wz = angular_vel
        roll, pitch, yaw = rpy
        # 使用旋转矩阵导数（简化示例）
        J = np.array([[1, 0, -wy*dt],
                    [wy*dt, 1, wx*dt],
                    [wz*dt, -wz*dt, 1]])
        return J

    def measurement_jacobian(self, state):
        # Compute the Jacobian of the measurement function
        roll, pitch, yaw = state[3:6]
        vx_body = self.dvl_data['velocity'].x
        vy_body = self.dvl_data['velocity'].y
        
        J = np.eye(9)
        J[0, 0] = 1
        J[1, 1] = 1
        J[2, 2] = 1
        J[3, 3] = 1
        J[4, 4] = 1
        J[5, 5] = 1
        J[6, 6] = np.cos(yaw)
        J[6, 7] = -np.sin(yaw)
        J[7, 6] = np.sin(yaw)
        J[7, 7] = np.cos(yaw)
        J[8, 8] = 1
        
        return J

    def measurement_covariance(self, state, P, R):
        # Compute the measurement covariance
        H = self.measurement_jacobian(state)
        return np.dot(np.dot(H, P), H.T) + R

    def kalman_gain(self, P, H, S):
        # Compute the Kalman gain
        return np.dot(np.dot(P, H.T), np.linalg.inv(S))

if __name__ == '__main__':
    node = UnscentedKalmanFilterNode()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.unscented_kalman_filter()
        rate.sleep()