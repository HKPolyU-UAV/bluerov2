#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, FluidPressure
from uuv_sensor_ros_plugins_msgs.msg import DVL
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ExtendedKalmanFilterNode:
    def __init__(self):
        rospy.init_node('ekf_sensor_fusion', anonymous=True)
        
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

        # EKF variables
        self.state = np.array([-12.0, 0.0, -95.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [x, y, z, roll, pitch, yaw, vx, vy, vz]
        self.P = np.eye(9) * 0.1  # State covariance matrix
        self.Q = np.eye(9) * 0.05  # Process noise covariance
        self.R = np.eye(9) * 15  # Measurement noise covariance

        # Subscribe to sensor topics
        rospy.Subscriber("/bluerov2/imu", Imu, self.imu_callback)
        rospy.Subscriber("/bluerov2/dvl", DVL, self.dvl_callback)
        rospy.Subscriber("/bluerov2/pressure", FluidPressure, self.pressure_callback)

        # Publish fused odometry and path
        self.odom_pub = rospy.Publisher("/bluerov2/fused_odom_ekf", Odometry, queue_size=10)
        self.path_pub = rospy.Publisher("/bluerov2/fused_trajectory_ekf", Path, queue_size=10)
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

    def predict(self, dt):
        # State transition function
        roll, pitch, yaw = self.state[3:6]
        vx, vy, vz = self.state[6:9]

        # Update state transition matrix F
        self.F = np.eye(9)
        self.F[0, 6] = dt
        self.F[1, 7] = dt
        self.F[2, 8] = dt

        # Predict state
        self.state[0] += vx * dt
        self.state[1] += vy * dt
        self.state[2] += vz * dt

        # Predict covariance
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, measurement):
        # Measurement function
        H = np.eye(9)

        # Kalman gain
        S = np.dot(H, np.dot(self.P, H.T)) + self.R
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))

        # Update state and covariance
        y = measurement - np.dot(H, self.state)
        self.state = self.state + np.dot(K, y)
        self.P = self.P - np.dot(K, np.dot(H, self.P))

    def fuse_sensors(self):
        if None in [self.imu_data, self.dvl_data, self.pressure_data]:
            return
            
        try:
            # Time synchronization (using IMU time)
            current_time = self.imu_data['timestamp']
            dt = (current_time - self.last_time).to_sec()
            if dt <= 0:
                return

            # Predict step
            self.predict(dt)

            # Measurement update
            orientation = self.imu_data['orientation']
            roll, pitch, yaw = euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
            vx_body = self.dvl_data['velocity'].x
            vy_body = self.dvl_data['velocity'].y
            vz_body = self.dvl_data['velocity'].z
            ax = self.imu_data['linear_acc'].x
            ay = self.imu_data['linear_acc'].y
            az = self.imu_data['linear_acc'].z

            # Convert body frame velocity to world frame
            vx_world = vx_body * np.cos(yaw) - vy_body * np.sin(yaw)
            vy_world = vx_body * np.sin(yaw) + vy_body * np.cos(yaw)
            vz_world = vz_body

            # Measurement vector
            measurement = np.array([
                self.position_x, self.position_y, self.pressure_data['depth'],
                roll, pitch, yaw,
                vx_world, vy_world, vz_world
            ])
            self.update(measurement)

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
            odom.pose.pose.orientation = orientation
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
            t.transform.rotation = orientation
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

if __name__ == '__main__':
    node = ExtendedKalmanFilterNode()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.fuse_sensors()
        rate.sleep()