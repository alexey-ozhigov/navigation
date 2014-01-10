#!/usr/bin/python

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from math import cos, sin, pi

MARKER_TOPIC = 'test_path_vis'

def init():
    rospy.init_node('test_path')
    return rospy.Publisher(MARKER_TOPIC, MarkerArray)

def get_ith_phi(phi_start, phi_end, Npoints, i):
    return phi_start + (phi_end - phi_start) * 1.0 * (Npoints - i) / Npoints

def get_marker_array():
    R = 1
    Rorient = 0.6
    Npoints = 50
    Xstep = 0.1
    phi_start = 0
    phi_end = pi
    ma = MarkerArray()
    for i in range(0, Npoints):
        m = Marker()
        m.header.frame_id = "/map"
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.id = i
        phi = get_ith_phi(phi_start, phi_end, Npoints, i)
        p1 = Point()
        p1.x = i*Xstep#R*cos(phi)
        p1.y = R*sin(phi)
        p1.z = 0
        p2 = Point()
        p2.x = p1.x + Rorient*cos(phi)
        p2.y = p1.y + Rorient*sin(phi)
        p2.z = 0
        m.points.append(p1)
        m.points.append(p2)
        m.scale.x = 0.1
        m.scale.y = 0.1
        m.scale.z = 0.1
        m.color.a = 1.0
        #m.pose.position.x = R*cos(phi)
        #m.pose.position.y = R*sin(phi)
        #m.pose.position.z = 0
        #m.pose.orientation.z = phi
        #m.pose.orientation.w = 1.0
        ma.markers.append(m)
    return ma

    
def run_test(pub):
    while not rospy.is_shutdown():
        marker_array = get_marker_array()
        pub.publish(marker_array)
        rospy.sleep(0.1)


if __name__ == '__main__':
    pub = init()
    run_test(pub)
