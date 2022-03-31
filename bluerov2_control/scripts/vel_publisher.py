#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
'''
def position_subscriber():
    sub = rospy.Subscriber('/bluerov2/pose_gt', Odometry, )
    rospy.init_node('position_subscriber')
    rate = rospy.Rate(10)
'''
'''
def ori_publisher():
    pub = rospy.Publisher('/bluerov2/cmd_vel', Twist, queue_size=1)
    rospy.init_node('ori_publisher')
    rate = rospy.Rate(20)

    move_angular = Twist()
    move_angular.angular.z = 0.0
    
    degree_aim = math.radians(45)
    degree_current = 0

    while not rospy.is_shutdown():
        
        while degree_current < degree_aim:
            pub.publish(move_angular)
            #degree_current = degree_current + 0.05
        
        pub.publish(move_angular)
        rate.sleep()
'''
def vel_publisher():
    pub = rospy.Publisher('/bluerov2/cmd_vel', Twist, queue_size=1)
    rospy.init_node('vel_publisher')
    rate = rospy.Rate(20)

    move_linear = Twist()
    move_linear.angular.z = 0.5

    degree_aim = math.radians(45)
    degree_current = 0

    while not rospy.is_shutdown():
        
        pub.publish(move_linear)
        rate.sleep()
    

if __name__ == '__main__':
    
    try:
        #ori_publisher()
        vel_publisher()
    except rospy.ROSInterruptException:
        pass
    