#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Point

roll = pitch = yaw = 0.0    # current roll, pitch, yaw
coord_x = coord_y = coord_z = 0.0   #current coordinate x, y, z

kp = 0.5
goal = Point()
goal.x = 8
goal.y = 8

def get_odom(msg):
    global roll, pitch, yaw
    global coord_x, coord_y, coord_z

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    position_q = msg.pose.pose.position
    (coord_x, coord_y, coord_z) = [position_q.x, position_q.y, position_q.z]


rospy.init_node('vel_control')
sub = rospy.Subscriber('/bluerov2/pose_gt', Odometry, get_odom)
pub = rospy.Publisher('bluerov2/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10)
move = Twist()

while not rospy.is_shutdown():
    yaw_deg = yaw/math.pi*180
    inc_x = goal.x - coord_x
    inc_y = goal.y - coord_y
    goal_yaw = math.atan2(inc_y, inc_x)
    goal_yaw_deg = goal_yaw/math.pi*180
    distance = math.sqrt(inc_x*inc_x + inc_y*inc_y)

    if abs(goal_yaw_deg-yaw_deg) > 5:
        move.linear.x = 0
        move.angular.z = kp * (goal_yaw-yaw)
        print('Adjust angle -- target:', goal_yaw_deg, 'current:', yaw_deg, 'angular.z:', move.angular.z)
    else:
        move.linear.x = kp*distance
        move.angular.z = 0
        print('Move towards aim -- target:', goal.x, goal.y, 'current:', coord_x, coord_y, 'linear.x:', move.linear.x)
        #distance_counter = distance_counter + 0.5


    pub.publish(move)
    rate.sleep()
    

