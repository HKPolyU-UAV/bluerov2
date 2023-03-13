#!/usr/bin/env python
"""

From bluerov_ros_playground respository (https://github.com/patrickelectric/bluerov_ros_playground)
Credits: patrickelectric

"""

import rospy
from gazebo_msgs.msg import ModelState
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from tf import transformations


class SITL(object):
    """Class to handle with SITL

    Attributes:
        pub (TYPE): ROS publisher
        sub (TYPE): ROS subscriber
    """

    def __init__(self):
        super(SITL, self).__init__()
        self._pose = PoseStamped()

        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self._pose_callback)
        self._pub = rospy.Publisher('/gazebo/set_model_state', ModelState)

    def _pose_callback(self, msg):
        self._pose = msg

    def run(self):
        """ Send SITL information to gazebo: Pose from Pixhawk
        """
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():

            # Get ROV position and send it to gazebo
            try:

                model_state = ModelState()
                model_state.model_name = 'bluerov2'
                # Set position
                model_state.pose.position.x = self._pose.pose.position.x
                model_state.pose.position.y = self._pose.pose.position.y
                model_state.pose.position.z = self._pose.pose.position.z

                Q = [q for q in self._pose.pose.orientation.x, self._pose.pose.orientation.y,
                                self._pose.pose.orientation.z, self._pose.pose.orientation.w]
                q1 = transformations.quaternion_from_euler(0,0,0,'sxyz')
                q = transformations.quaternion_multiply(Q, q1)

                model_state.pose.orientation.x = q[0]
                model_state.pose.orientation.y = q[1]
                model_state.pose.orientation.z = q[2]
                model_state.pose.orientation.w = q[3]

                self._pub.publish(model_state)
            except Exception as error:
                rospy.logerr('{}: Get data error {}'.format(rospy.get_name(), error))


if __name__ == "__main__":
    try:
        rospy.init_node('bluerov_sitl', log_level=rospy.INFO)
    except rospy.ROSInterruptException as error:
        rospy.logerr('{}: pubs error with ROS {}'.format(rospy.get_name(), error))
        exit(1)
    bluerov_sitl = SITL()
    bluerov_sitl.run()
