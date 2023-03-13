#!/usr/bin/env python

import rospy

from mavros_msgs.msg import ManualControl  # message control to ardusub
from sensor_msgs.msg import Joy  # joystick controls
from geometry_msgs.msg import WrenchStamped, Vector3Stamped
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import numpy as np
from bluerov2_control.msg import ControlMode
from bluerov2_control.srv import SetControlMode, SetControlModeRequest


class MavrosInterface(object):

    """
    Class to handle control messages to MAVROS
    - Maps Wrench to ManualControl Message
    - If controller is IDLE or OFF, then route Joystick axes to ManualControl message
    """

    def __init__(self):
        self._controller_state = ControlMode.OFF
        self._btn_remap = rospy.get_param("~mappings", {})
        self._wrench_polynomial = rospy.get_param("~wrench_polynomial", {"x": [1, 0],
                                                                         "y": [1, 0],
                                                                         "z": [1, 0],
                                                                         "r": [1, 0]})
        self._switch_trigger = 0
        self._joy_polynomial = {"x": [1000, 0],
                               "y": [1000, 0],
                               "z": [500, 500],
                               "r": [1000, 0]}
        self._man = ManualControl()  # manual control message
        self._man.z = 500.0  # default 0 throttle value
        self._man_pub = rospy.Publisher("mavros/manual_control/send", ManualControl, queue_size=10)
        self._tf_buff = Buffer()
        TransformListener(self._tf_buff)  # listen for TFs to transform between wrench and base_link
        self._serv = rospy.ServiceProxy("controller/set_control_mode", SetControlMode)
        rospy.Subscriber('controller/mode', ControlMode, self._update_mode)
        rospy.Subscriber('joy', Joy, self._joy_callback)  # we listen on joystick for button commands
        rospy.Subscriber('wrench/target', WrenchStamped, self._wrench_callback)  # listen on input_stamped for commanded wrench

    def _update_mode(self, mode):
        self._controller_state = mode.mode

    def _trigger_manual(self):
        # transition to direct teleop with FCU
        rospy.logdebug("SWITCH STATE CALLED")
        self._serv.wait_for_service()
        req = SetControlModeRequest()
        req.mode.mode = req.mode.OFF
        res = self._serv.call()
        if not res.success:
            rospy.logerr("{} | Could not turn controller off!".format(rospy.get_name()))

    def _wrench_callback(self, msg):
        """
        Commanded wrench is converted to joystick gains and sent to manual control topic
        """
        # if not msg.header.frame_id.endswith("frd"):
        #     forces = Vector3Stamped()
        #     forces.header = msg.header
        #     forces.vector = msg.wrench.force
        #     torques = Vector3Stamped()
        #     torques.header = msg.header
        #     torques.vector = msg.wrench.torque
        #     forces = self._tf_buff.transform(forces, forces.header.frame_id+"_frd", rospy.Duration(1))
        #     torques = self._tf_buff.transform(torques, torques.header.frame_id+"_frd", rospy.Duration(1))
        #     msg.wrench.force = forces.vector
        #     msg.wrench.torque = torques.vector
        # rospy.loginfo_throttle(1.0, msg)
        # If the controller state is ON, then map wrench to joystick gains and send to ROV
        if self._controller_state != ControlMode.OFF:
            self._man.x = np.clip(np.polyval(self._wrench_polynomial["x"], msg.wrench.force.x), -1000.0, 1000.0)
            self._man.y = -np.clip(np.polyval(self._wrench_polynomial["y"], msg.wrench.force.y), -1000.0, 1000.0)
            self._man.z = np.clip(np.polyval(self._wrench_polynomial["z"], msg.wrench.force.z), -1000.0, 1000.0)
            self._man.r = -np.clip(np.polyval(self._wrench_polynomial["r"], msg.wrench.torque.z), -1000.0, 1000.0)

    def _joy_callback(self, msg):
        # First look at the buttons and decide if the manual override button has been triggered
        btns = msg.buttons
        if btns[self._btn_remap['switch']] and not self._switch_trigger:
            self._trigger_manual()
        self._switch_trigger = btns[self._btn_remap['switch']]
        # booleans in LSB order, taken from default joystick mappings for ArduSub
        bools = [bool(i) for i in [btns[self._btn_remap['depth_hold']] if self._btn_remap['depth_hold'] >= 0 else 0,
                                   0,
                                   btns[self._btn_remap['manual']] if self._btn_remap['manual'] >= 0 else 0,
                                   btns[self._btn_remap['stabilize']] if self._btn_remap['stabilize'] >= 0 else 0,
                                   btns[self._btn_remap['mount_dec']] if self._btn_remap['mount_dec'] >= 0 else 0,
                                   btns[self._btn_remap['mount_inc']] if self._btn_remap['mount_inc'] >= 0 else 0,
                                   0,
                                   0,
                                   btns[self._btn_remap['disarm']] if self._btn_remap['disarm'] >= 0 else 0,
                                   btns[self._btn_remap['arm']] if self._btn_remap['arm'] >= 0 else 0,
                                   btns[self._btn_remap['mount_center']] if self._btn_remap['mount_center'] >= 0 else 0,
                                   0,
                                   btns[self._btn_remap['light_inc']] if self._btn_remap['light_inc'] >= 0 else 0,
                                   btns[self._btn_remap['light_dec']] if self._btn_remap['light_dec'] >= 0 else 0,
                                   btns[self._btn_remap['gain_inc']] if self._btn_remap['gain_inc'] >= 0 else 0,
                                   btns[self._btn_remap['gain_dec']] if self._btn_remap['gain_dec'] >= 0 else 0,
                                   ]]
        bools.reverse()
        out = 0
        for bit in bools:
            out = (out << 1) | bit
        self._man.buttons = out
        # If controller is in OFF state, then forward joystick axes messages
        if self._controller_state == ControlMode.OFF:
            self._man.x = np.clip(np.polyval(self._joy_polynomial["x"], msg.axes[0]), -1000.0, 1000.0)
            self._man.y = np.clip(np.polyval(self._joy_polynomial["y"], msg.axes[1]), -1000.0, 1000.0)
            self._man.z = np.clip(np.polyval(self._joy_polynomial["z"], msg.axes[2]), -1000.0, 1000.0)
            self._man.r = np.clip(np.polyval(self._joy_polynomial["r"], msg.axes[3]), -1000.0, 1000.0)
        rospy.logdebug('{} manual: {}'.format(rospy.get_name(), out))

    def run(self):
        hz = rospy.Rate(10)  # rate to send to the beast
        while not rospy.is_shutdown():
            try:
                self._man.header.stamp = rospy.Time.now()
                self._man_pub.publish(self._man)
                hz.sleep()
            except Exception as error:
                rospy.logerr('{} rc error: {}'.format(rospy.get_name(), error))


if __name__ == "__main__":
    try:
        rospy.init_node('mavros_interface')
        interface = MavrosInterface()
        interface.run()
    except rospy.ROSInterruptException:
        pass