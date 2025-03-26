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
import ceres_solver as cs

class SensorFusionNode:
    def __init__(self):
        rospy.init_node('blueROV2_sensor_fusion', anonymous=True)
        
        # Initialize sensor data
        self.imu_data = None
        self.dvl_data = None
        self.pressure_data = None
        self.last_time = rospy.Time.now()

        # Initialize trajectory and position
        self.path = Path()
        self.path.header.frame_id = "map"
        self.position_x = -12.0
        self.position_y = 0.0

        self.gt_path = Path()
        self.gt_path.header.frame_id = "map"
        
        # Subscribe to sensor topics
        rospy.Subscriber("/bluerov2/imu", Imu, self.imu_callback)
        rospy.Subscriber("/bluerov2/dvl", DVL, self.dvl_callback)
        rospy.Subscriber("/bluerov2/pressure", FluidPressure, self.pressure_callback)
        rospy.Subscriber("/bluerov2/pose_gt", Odometry, self.gt_callback)

        # Publish fused odometry and path
        self.odom_pub = rospy.Publisher("/bluerov2/fused_odom", Odometry, queue_size=10)
        self.path_pub = rospy.Publisher("/bluerov2/fused_trajectory", Path, queue_size=10)
        self.gt_path_pub = rospy.Publisher("/bluerov2/gt_trajectory", Path, queue_size=10)
        self.tf_broadcaster = TransformBroadcaster()
        
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

            # Get body-frame velocity from DVL
            vx_body = self.dvl_data['velocity'].x
            vy_body = self.dvl_data['velocity'].y

            # Convert to world frame
            vx_world = vx_body * np.cos(yaw) - vy_body * np.sin(yaw)
            vy_world = vx_body * np.sin(yaw) + vy_body * np.cos(yaw)

            # Position integration
            self.position_x += vx_world * dt
            self.position_y += vy_world * dt
            z = self.pressure_data['depth']

            # Construct Odometry message
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.position_x
            odom.pose.pose.position.y = self.position_y
            odom.pose.pose.position.z = z
            odom.pose.pose.orientation = orientation
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
            t.transform.translation.z = z
            t.transform.rotation = orientation
            self.tf_broadcaster.sendTransform(t)

            # Update trajectory
            pose_stamped = PoseStamped()
            pose_stamped.header = odom.header
            pose_stamped.pose = odom.pose.pose
            self.path.poses.append(pose_stamped)
            self.path_pub.publish(self.path)
            self.odom_pub.publish(odom)
            
            self.last_time = current_time
            
        except Exception as e:
            rospy.logerr("Fusion error: %s" % str(e))

    def factor_graph_fusion(self):
        # Create a Ceres factor graph
        graph = cs.Problem()

        # Add poses as variables
        pose_var = graph.AddParameterBlock("pose", 7)

        # Add IMU factor
        imu_factor = cs.ImuFactor(self.imu_data['angular_vel'], self.imu_data['timestamp'])
        graph.AddResidualBlock(imu_factor, None, pose_var)

        # Add DVL factor
        dvl_factor = cs.DvlFactor(self.dvl_data['velocity'], self.dvl_data['timestamp'])
        graph.AddResidualBlock(dvl_factor, None, pose_var)

        # Add pressure factor
        pressure_factor = cs.PressureFactor(self.pressure_data['depth'], self.pressure_data['timestamp'])
        graph.AddResidualBlock(pressure_factor, None, pose_var)

        # Optimize the factor graph
        options = cs.SolverOptions()
        options.max_num_iterations = 100
        summary = cs.Solve(options, graph)

        # Extract the optimized pose
        optimized_pose = graph.GetParameterBlock("pose")

        # Publish the fused odometry and path
        odom = Odometry()
        odom.header.stamp = self.imu_data['timestamp']
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = optimized_pose[0]
        odom.pose.pose.position.y = optimized_pose[1]
        odom.pose.pose.position.z = optimized_pose[2]
        odom.pose.pose.orientation.x = optimized_pose[3]
        odom.pose.pose.orientation.y = optimized_pose[4]
        odom.pose.pose.orientation.z = optimized_pose[5]
        odom.pose.pose.orientation.w = optimized_pose[6]
        odom.twist.twist.linear.x = self.dvl_data['velocity'].x
        odom.twist.twist.linear.y = self.dvl_data['velocity'].y
        odom.twist.twist.angular = self.imu_data['angular_vel']

        pose_stamped = PoseStamped()
        pose_stamped.header = odom.header
        pose_stamped.pose = odom.pose.pose
        self.path.poses.append(pose_stamped)
        self.path_pub.publish(self.path)
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    node = SensorFusionNode()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.fuse_sensors()
        node.factor_graph_fusion()
        rate.sleep()