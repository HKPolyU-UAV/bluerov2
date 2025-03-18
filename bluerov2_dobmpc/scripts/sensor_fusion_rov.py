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

class SensorFusionNode:
    def __init__(self):
        rospy.init_node('blueROV2_sensor_fusion', anonymous=True)
        
        # 初始化传感器数据
        self.imu_data = None
        self.dvl_data = None
        self.pressure_data = None
        self.last_time = rospy.Time.now()

        # 初始化轨迹和位置
        self.path = Path()
        # self.path.header.frame_id = "odom"
        self.path.header.frame_id = "world"
        self.position_x = 10.0
        self.position_y = 20.0

        self.gt_path = Path()
        self.gt_path.header.frame_id = "world"
        
        # 订阅传感器话题
        rospy.Subscriber("/bluerov2/imu", Imu, self.imu_callback)
        rospy.Subscriber("/bluerov2/dvl", DVL, self.dvl_callback)
        rospy.Subscriber("/bluerov2/pressure", FluidPressure, self.pressure_callback)
        rospy.Subscriber("/bluerov2/pose_gt", Odometry, self.gt_callback)

        # 发布融合后的位姿和轨迹
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
            # 时间同步（以IMU时间为准）
            current_time = self.imu_data['timestamp']
            dt = (current_time - self.last_time).to_sec()
            if dt <= 0:
                return

            # 从IMU获取姿态
            orientation = self.imu_data['orientation']
            _, _, yaw = euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w
            ])

            # 从DVL获取机体坐标系速度
            vx_body = self.dvl_data['velocity'].x
            vy_body = self.dvl_data['velocity'].y

            # 转换到世界坐标系
            vx_world = vx_body * np.cos(yaw) - vy_body * np.sin(yaw)
            vy_world = vx_body * np.sin(yaw) + vy_body * np.cos(yaw)

            # 位置积分
            self.position_x += vx_world * dt
            self.position_y += vy_world * dt
            z = self.pressure_data['depth']

            # 构建Odometry消息
            odom = Odometry()
            odom.header.stamp = current_time
            # odom.header.frame_id = "odom"
            odom.header.frame_id = "world"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.position_x
            odom.pose.pose.position.y = self.position_y
            odom.pose.pose.position.z = z
            odom.pose.pose.orientation = orientation
            odom.twist.twist.linear.x = vx_world
            odom.twist.twist.linear.y = vy_world
            odom.twist.twist.angular = self.imu_data['angular_vel']

            # 发布TF
            t = TransformStamped()
            t.header.stamp = current_time
            # t.header.frame_id = "odom"
            t.header.frame_id = "world"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.position_x
            t.transform.translation.y = self.position_y
            t.transform.translation.z = z
            t.transform.rotation = orientation
            self.tf_broadcaster.sendTransform(t)

            # 更新轨迹
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