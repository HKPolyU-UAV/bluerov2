#!/usr/bin/env python
import rospy
from uuv_sensor_ros_plugins_msgs.msg import DVL
import numpy as np

def add_noise(vel_msg):
    # 添加高斯噪声
    noise = np.random.normal(0, 0.1)  # sigma=0.05
    vel_msg.velocity.x += noise
    return vel_msg

def main():
    rospy.init_node('dvl_noise_adder')
    sub = rospy.Subscriber('/bluerov2/dvl', DVL, add_noise)
    pub = rospy.Publisher('/bluerov2/dvl_noise', DVL, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()