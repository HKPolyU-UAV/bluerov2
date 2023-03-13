#!/usr/bin/env python

import rospy
from uuv_control_msgs.srv import InitWaypointSet, InitWaypointSetRequest, InitWaypointSetResponse
from uuv_control_msgs.msg import Waypoint
from bluerov2_control.srv import ConvertGeoPointsRequest, ConvertGeoPoints
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import String, Time
from tf2_ros import Buffer, TransformListener
import sys
import os
import re
import tf2_geometry_msgs  # this is a necessary import, don't be fooled


def parse_iver_format(string):
    values = re.split(', | ',string)
    return values


def parse_request(mis, utm_frame_id="utm", radius_of_acceptance=5.0):
    with open(mis, "r") as f:
        while not f.readline().startswith("START"):
            pass
        geopoints = []
        headings = []
        speeds = []
        line = f.readline().rstrip()
        while not line.startswith("END"):
            values = line.split(";")
            values = [value.strip() for value in values]
            params = parse_iver_format(values[5])
            zvar = -float(params[0][1:])*0.3048 if params[0][0] == 'D' else float(params[0][1:])*0.3048
            geopoints.append(GeoPoint(float(values[1]),float(values[2]), zvar))
            headings.append(float(values[4]))
            speeds.append(float(params[-1][1:])*0.5144)
            line = f.readline().rstrip()
        # while not f.readline().startswith("START VEHICLE"):
        #     pass
        # line = f.readline().rstrip()
        # while not line.startswith("END VEHICLE"):
        #     if line.startswith("UVC=WPRadius"):
        #         roas.append(float(line.split("=")[-1]))
        #         break

    req = ConvertGeoPointsRequest()
    req.geopoints = geopoints
    p = rospy.ServiceProxy("convert_points", ConvertGeoPoints)
    p.wait_for_service()
    res = p(req)
    wps = []
    for point, heading, speed in zip(res.utmpoints, headings, speeds):
        wp = Waypoint()
        wp.point.x = point.x
        wp.point.y = point.y
        wp.point.z = point.z
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = utm_frame_id
        wp.radius_of_acceptance = radius_of_acceptance
        wp.max_forward_speed = speed
        wp.use_fixed_heading = False
        wp.heading_offset = 0.0
        wps.append(wp)
    req = InitWaypointSetRequest()
    req.start_time = Time(rospy.Time.from_sec(0))
    req.start_now = True
    req.waypoints = wps
    req.max_forward_speed = max(speeds)
    req.interpolator = String("lipb")
    req.heading_offset = 0
    req.max_forward_speed = max(speeds)
    req.waypoints = wps
    return req

if __name__=="__main__":
    rospy.init_node("set_iver_waypoints")
    if len(sys.argv) < 2:
        rospy.logerr('Invalid number of parameters\nusage: '
                     './import_iver_mis.py '
                     'iver_mission.mis (REQUIRED) '
                     '__ns:="name" (OPTIONAL) '
                     '_radius:="radius_of_acceptance (m)" (OPTIONAL) '
                     '_frame_id:="frame name of utm" (OPTIONAL)')
        sys.exit(0)
    else:
        mission_filename = sys.argv[1] if sys.argv[1].endswith(".mis") else sys.argv[1]+".mis"
        if not os.path.exists(mission_filename):
            rospy.logerr("File not found: {}".format(mission_filename))
        else:
            rospy.loginfo("Parsing Iver Mission File: {}".format(mission_filename))
    roa = rospy.get_param("~radius", 5.0)
    frame_id = rospy.get_param("~utm_frame_id", "utm")
    req = parse_request(mission_filename, utm_frame_id=frame_id, radius_of_acceptance=roa)
    proxy = rospy.ServiceProxy("load_waypoints", InitWaypointSet)
    try:
        proxy.wait_for_service(timeout=1.0)
        res = proxy(req)
        if res.success:
            rospy.loginfo("IVER Waypoints Set, Executing.")
        else:
            rospy.logerr("IVER Waypoints Error.")
    except Exception as e:
        rospy.logerr("{} | {}".format(rospy.get_name(), e.message))
