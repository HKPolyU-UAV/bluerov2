#!/usr/bin/env python3
import rospy
import time
from gazebo_msgs.srv import ApplyJointEffort

def attach_light():
    rospy.init_node('attach_light')

    # Wait for the /gazebo/apply_joint_effort service to become available
    rospy.wait_for_service('/gazebo/apply_joint_effort')

    # Create a proxy for the /gazebo/apply_joint_effort service
    apply_joint_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)

    # Call the /gazebo/apply_joint_effort service to create a fixed joint between the bluerov2/base_link link and the light_source1/link link
    apply_joint_effort('bluerov2/base_link', 'light_source1/link', 0, rospy.Time(), rospy.Duration())

if __name__ == '__main__':
    try:
        time.sleep(20)
        attach_light()
    except rospy.ROSInterruptException:
        pass