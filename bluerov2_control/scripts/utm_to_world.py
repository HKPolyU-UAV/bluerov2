#!/usr/bin/env python


import rospy
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geographic_msgs.msg import GeoPoint
from geodesy import utm


if __name__=="__main__":
    rospy.init_node("utm_to_world")
    datum = map(float, rospy.get_param("~datum", []).strip('][').split(','))
    datum = GeoPoint(*map(float, datum))
    utmpoint = utm.fromMsg(datum)
    broadcaster = StaticTransformBroadcaster()
    tf = TransformStamped()
    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = "utm"
    tf.child_frame_id = "world"
    tf.transform.translation.x = utmpoint.easting
    tf.transform.translation.y = utmpoint.northing
    tf.transform.translation.z = utmpoint.altitude
    tf.transform.rotation.w = 1.0
    broadcaster.sendTransform(tf)
    rospy.spin()
