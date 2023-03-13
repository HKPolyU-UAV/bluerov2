#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Joy
from mavros_msgs.msg import ManualControl


class SITLTeleop(object):

    """Class to handle with gazebo teleop
    Attributes:
        pub (TYPE): ROS publisher
        sub (TYPE): ROS subscriber
    """

    def __init__(self):
        super(SITLTeleop, self).__init__()
        self.btn_remap = self.load_mappings()

        self.man = ManualControl()
        self.man.z = 500.0
        self.pub = rospy.Publisher("mavros/manual_control/send", ManualControl, queue_size=10)

        rospy.Subscriber('joy', Joy, self.joy_callback)

    def load_mappings(self):
        return rospy.get_param("~mappings", {})

    def bool2int(self, bools):
        return

    def joy_callback(self, msg):
        self.man.header.frame_id = msg.header.frame_id
        self.man.x = msg.axes[0] * 1000.0
        self.man.y = msg.axes[1] * 1000.0
        self.man.z = msg.axes[2] * 1000.0
        self.man.r = msg.axes[3] * 1000.0
        btns = msg.buttons
        # booleans in LSB order, taken from default joystick mappings for ArduSub
        bools = [bool(i) for i in [btns[self.btn_remap['depth_hold']] if self.btn_remap['depth_hold'] >= 0 else 0,
                                   0,
                                   btns[self.btn_remap['manual']] if self.btn_remap['manual'] >= 0 else 0,
                                   btns[self.btn_remap['stabilize']] if self.btn_remap['stabilize'] >= 0 else 0,
                                   btns[self.btn_remap['mount_dec']] if self.btn_remap['mount_dec'] >= 0 else 0,
                                   btns[self.btn_remap['mount_inc']] if self.btn_remap['mount_inc'] >= 0 else 0,
                                   0,
                                   0,
                                   btns[self.btn_remap['disarm']] if self.btn_remap['disarm'] >= 0 else 0,
                                   btns[self.btn_remap['arm']] if self.btn_remap['arm'] >= 0 else 0,
                                   btns[self.btn_remap['mount_center']] if self.btn_remap['mount_center'] >= 0 else 0,
                                   0,
                                   btns[self.btn_remap['light_inc']] if self.btn_remap['light_inc'] >= 0 else 0,
                                   btns[self.btn_remap['light_dec']] if self.btn_remap['light_dec'] >= 0 else 0,
                                   btns[self.btn_remap['gain_inc']] if self.btn_remap['gain_inc'] >= 0 else 0,
                                   btns[self.btn_remap['gain_dec']] if self.btn_remap['gain_dec'] >= 0 else 0,
                                   ]]
        self.man.buttons = int(''.join(str(int(i)) for i in reversed(bools)), 2)
        rospy.logdebug('{} manual: {}'.format(rospy.get_name(), int(''.join(str(int(i)) for i in reversed(bools)), 2)))

    def run(self):
        """ Run Gazebo Teleop
        """
        hz = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.man.header.stamp = rospy.Time.now()
                self.pub.publish(self.man)
                hz.sleep()
            except Exception as error:
                rospy.logerr('{} rc error: {}'.format(rospy.get_name(), error))


if __name__ == "__main__":
    try:
        rospy.init_node('sitl_teleop')
    except rospy.ROSInterruptException:
        exit(1)
    sitl_teleop = SITLTeleop()
    sitl_teleop.run()