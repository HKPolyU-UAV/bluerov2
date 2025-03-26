#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from sensor_msgs.msg import Imu, FluidPressure
from uuv_sensor_ros_plugins_msgs.msg import DVL
from nav_msgs.msg import Odometry, Path
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import gtsam

class SensorFusionNode:
    def __init__(self):
        rospy.init_node('blueROV2_sensor_fusion', anonymous=True)
        
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

        self.gt_path = Path()
        self.gt_path.header.frame_id = "map"
        
        # Subscribe to sensor topics
        rospy.Subscriber("/bluerov2/imu", Imu, self.imu_callback)
        rospy.Subscriber("/bluerov2/dvl", DVL, self.dvl_callback)
        rospy.Subscriber("/bluerov2/pressure", FluidPressure, self.pressure_callback)
        rospy.Subscriber("/bluerov2/pose_gt", Odometry, self.gt_callback)

        # Publish fused odometry and path
        self.odom_pub = rospy.Publisher("/bluerov2/fused_odom_fgo", Odometry, queue_size=10)
        self.path_pub = rospy.Publisher("/bluerov2/fused_trajectory_fgo", Path, queue_size=10)
        self.gt_path_pub = rospy.Publisher("/bluerov2/gt_trajectory_fgo", Path, queue_size=10)
        self.tf_broadcaster = TransformBroadcaster()

        # Initialize factor graph
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        self.pose_key_counter = 0
        self.pose_key = gtsam.symbol('x', self.pose_key_counter)
        self.velocity_key = gtsam.symbol('v', 0)
        self.bias_key = gtsam.symbol('b', 0)

        # Add prior factors
        prior_pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(self.position_x, self.position_y, self.position_z))
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]))
        self.graph.add(gtsam.PriorFactorPose3(self.pose_key, prior_pose, prior_noise))
        self.initial_estimate.insert(self.pose_key, prior_pose)

        prior_velocity = gtsam.Point3(0, 0, 0)
        velocity_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        self.graph.add(gtsam.PriorFactorPoint3(self.velocity_key, prior_velocity, velocity_noise))
        self.initial_estimate.insert(self.velocity_key, prior_velocity)

        prior_bias = gtsam.imuBias.ConstantBias()
        bias_noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
        self.graph.add(gtsam.PriorFactorConstantBias(self.bias_key, prior_bias, bias_noise))
        self.initial_estimate.insert(self.bias_key, prior_bias)

    def imu_callback(self, msg):
        self.imu_data = {
            'angular_vel': msg.angular_velocity,
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
        
    def gt_callback(self, msg):
        # Update ground truth path
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.gt_path.poses.append(pose_stamped)
        self.gt_path_pub.publish(self.gt_path)

    def fuse_sensors(self):
        if None in [self.imu_data, self.dvl_data, self.pressure_data]:
            return
            
        try:
            # Time synchronization (using IMU time)
            current_time = self.imu_data['timestamp']
            dt = (current_time - self.last_time).to_sec()
            if dt <= 0:
                return

            # Get orientation from IMU
            orientation = self.imu_data['orientation']
            _, _, yaw = euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w
            ])

            # Get body frame velocity from DVL
            vx_body = self.dvl_data['velocity'].x
            vy_body = self.dvl_data['velocity'].y

            # Convert to world frame
            vx_world = vx_body * np.cos(yaw) - vy_body * np.sin(yaw)
            vy_world = vx_body * np.sin(yaw) + vy_body * np.cos(yaw)

            # Integrate position
            self.position_x += vx_world * dt
            self.position_y += vy_world * dt
            self.position_z = self.pressure_data['depth']

            # Add factors to the graph
            pose = gtsam.Pose3(gtsam.Rot3(orientation.w, orientation.x, orientation.y, orientation.z), gtsam.Point3(self.position_x, self.position_y, self.position_z))
            new_pose_key = gtsam.symbol('x', self.pose_key_counter + 1)
            # self.graph.add(gtsam.BetweenFactorPose3(self.pose_key, new_pose_key, pose, gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]))))
            self.initial_estimate.insert(new_pose_key, pose)
            self.pose_key = new_pose_key
            self.pose_key_counter += 1

            # Optimize the graph
            self.optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate)
            result = self.optimizer.optimize()

            # Get the optimized pose
            optimized_pose = result.atPose3(self.pose_key)
            self.position_x = optimized_pose.x()
            self.position_y = optimized_pose.y()
            self.position_z = optimized_pose.z()
            orientation = optimized_pose.rotation().toQuaternion()

            # Create Odometry message
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.position_x
            odom.pose.pose.position.y = self.position_y
            odom.pose.pose.position.z = self.position_z
            odom.pose.pose.orientation = Quaternion(orientation.w(), orientation.x(), orientation.y(), orientation.z())
            odom.twist.twist.linear.x = vx_world
            odom.twist.twist.linear.y = vy_world
            odom.twist.twist.angular = self.imu_data['angular_vel']

            # Publish TF
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.position_x
            t.transform.translation.y = self.position_y
            t.transform.translation.z = self.position_z
            t.transform.rotation = Quaternion(orientation.w(), orientation.x(), orientation.y(), orientation.z())
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
    node = SensorFusionNode()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.fuse_sensors()
        rate.sleep()