#!/usr/bin/python

import rospy
from std_msgs.msg import *
from move_base.srv import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from copy import deepcopy

def optimize_global_plan_pose(path):
    N = len(path.poses)
    phi_start = path.poses[0].pose.orientation.z
    phi_end = path.poses[N-1].pose.orientation.z
    phi_delta = 1.0 * (phi_start - phi_end) / N
    path = Path()
    for i in range(1, len(path.poses)):
        point = path.poses[i]
        new_point = deepcopy(point)
        new_point.pose.orientation.z = phi_start + i * phi_delta
        path.poses.append(new_point)
    return path

def optimize_global_path_srv(req):
    return PathOptimizerResponse(optimize_global_plan_pose(req.path))

def run_server():
    rospy.init_node('global_path_optimizer')
    s = rospy.Service('global_path_optimizer', PathOptimizer, optimize_global_path_srv)
    rospy.spin()

if __name__ == '__main__':
    run_server()
