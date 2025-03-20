#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, FluidPressure
from uuv_sensor_ros_plugins_msgs.msg import DVL
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf.transformations as tf_trans
import numpy as np

class DeadReckoningNode:
    def __init__(self):
        rospy.init_node('dead_reckoning_node')

        # Initialize pose
        self.pose = PoseStamped()
        self.pose.header.frame_id = "world"
        self.pose.pose.position.x = 10.0
        self.pose.pose.position.y = 20.0
        self.pose.pose.position.z = -95.0
        self.pose.pose.orientation.w = 1.0
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = 0.0

        # Initialize velocity and depth
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.depth = 0.0

        # Subscribers
        rospy.Subscriber('/bluerov2/imu', Imu, self.imu_callback)
        rospy.Subscriber('/bluerov2/dvl', DVL, self.dvl_callback)
        rospy.Subscriber('/bluerov2/pressure', FluidPressure, self.pressure_callback)

        # Publisher
        self.pose_pub = rospy.Publisher('/bluerov2/estimated_pose', PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher('/bluerov2/estimated_path', Path, queue_size=10)

        # Initialize path
        self.path = Path()
        self.path.header.frame_id = "world"

        # Timer for updating pose
        self.last_time = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_pose)

    def imu_callback(self, msg):
        # Update orientation from IMU
        self.pose.pose.orientation = msg.orientation
        # Update angular velocity from IMU
        # self.twist.twist.angular = msg.angular_velocity

    def dvl_callback(self, msg):
        # Update velocity from DVL
        self.velocity = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        # self.twist.twist.linear = msg.velocity

    def pressure_callback(self, msg):
        # Update depth from pressure sensor
        # Assuming a simple conversion from pressure to depth
        self.depth = - (msg.fluid_pressure * 10000 - 101325) / (1000 * 9.81)

    def update_pose(self, event):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Get current orientation as a rotation matrix
        orientation_q = self.pose.pose.orientation
        orientation_matrix = tf_trans.quaternion_matrix([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

        # Transform velocity to world frame
        velocity_world = np.dot(orientation_matrix[:3, :3], self.velocity)

        # Update linear velocity
        # self.twist.twist.linear.x = velocity_world[0]
        # self.twist.twist.linear.y = velocity_world[1]
        # self.twist.twist.linear.z = velocity_world[2]

        # Update position
        self.pose.pose.position.x += velocity_world[0] * dt
        self.pose.pose.position.y += velocity_world[1] * dt
        # self.pose.pose.position.z = -self.depth  # Assuming depth is positive downwards
        self.pose.pose.position.z += velocity_world[2] * dt

        # Publish the estimated pose
        self.pose.header.stamp = current_time
        self.pose_pub.publish(self.pose)

        # Update and publish the path
        self.path.header.stamp = current_time
        pose_copy = PoseStamped()
        pose_copy.pose = self.pose.pose
        pose_copy.header = self.pose.header
        self.path.poses.append(pose_copy)
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    try:
        node = DeadReckoningNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass