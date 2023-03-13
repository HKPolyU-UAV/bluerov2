#!/usr/bin/env python
# -*- coding: utf-8 -*-


from geodesy import utm
from bluerov2_control.srv import ConvertGeoPoints, ConvertGeoPointsResponse
import rospy

"""This Node Simply Provides a Service to Convert Geographical Points (Latitude, Longitude, Altitude) into UTM 
Points. """


def convert_points(req):
    utms = [utm.fromMsg(msg) for msg in req.geopoints]
    res = ConvertGeoPointsResponse()
    res.utmpoints = [point.toPoint() for point in utms]
    return res


if __name__=="__main__":
    rospy.init_node("geodesy_to_local")
    serv = rospy.Service("convert_points", ConvertGeoPoints, convert_points)
    serv.spin()