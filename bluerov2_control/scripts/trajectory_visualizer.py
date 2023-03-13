#!/usr/bin/env python


import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from uuv_control_msgs.msg import WaypointSet
from std_msgs.msg import Header, ColorRGBA
from collections import deque
from bluerov2_control.cfg import marker_reconfigConfig
from dynamic_reconfigure.server import Server


class MarkerVisualization(object):
    def __init__(self):
        self._path_update_period = rospy.Duration.from_sec(rospy.get_param("~update_period", 1.0))
        self._path_buffer_len = rospy.get_param("~buffer_len", 100)
        self._path = Path(Header(0, rospy.Time.now(), ""), deque(maxlen=self._path_buffer_len))
        self._path_pub = rospy.Publisher("history", Path, queue_size=1)
        self._markers = MarkerArray()
        self._marker_pub = rospy.Publisher("waypoint_markers", MarkerArray, queue_size=1)
        self._dr_serv = Server(marker_reconfigConfig, self._reconfig)
        self._scale = Vector3(1,1,1)
        rospy.Subscriber("odometry", Odometry, self._update_vehicle_path)
        rospy.Subscriber("current_waypoints", WaypointSet, self._update_marker_list)

    def _reconfig(self, config, level):
        self._scale = Vector3(config["scale_x"],
                              config["scale_y"],
                              config["scale_z"])
        self._colors = ColorRGBA(config["color_r"],
                                 config["color_g"],
                                 config["color_b"],
                                 config["color_a"])
        self._modify_marker_list()
        return config

    def _modify_marker_list(self):
        for i, m in enumerate(self._markers.markers):
            m.action = m.MODIFY
            m.scale = self._scale
            m.color = self._colors
        self._send_markers()

    def _reset_markers(self):
        for m in self._markers.markers:
            m = Marker()
            m.action = m.DELETE
        self._send_markers()

    def _update_marker_list(self, msg):
        markers = MarkerArray()
        for i, wp in enumerate(msg.waypoints):
            m = Marker()
            m.header = wp.header
            m.pose = Pose(wp.point, Quaternion(0, 0, 0, 1))
            m.type = m.SPHERE
            m.action = m.ADD
            m.id = i
            m.scale = self._scale
            m.color = self._colors
            markers.markers.append(m)
        self._markers = markers
        self._send_markers()

    def _send_markers(self):
        self._marker_pub.publish(self._markers)

    def _update_vehicle_path(self, msg):
        if (msg.header.stamp - self._path.header.stamp) > self._path_update_period:
            self._path.poses.append(PoseStamped(msg.header, msg.pose.pose))
            self._path.header = msg.header
            self._path_pub.publish(self._path)


if __name__=="__main__":
    rospy.init_node("marker_manager_node")
    o = MarkerVisualization()
    rospy.spin()
