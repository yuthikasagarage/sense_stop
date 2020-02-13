#!/usr/bin/env python2
import rospy
import time
import sys
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np


def callback(data):
    ranges = data.ranges
    rangesarray = np.array(ranges, dtype=np.float)
    sample_indices = np.where(np.logical_and(
        rangesarray <= 2, rangesarray != np.float('inf')))
    quad_str = '0'
    if sample_indices[0].size != 0:
        indices = sample_indices[0]
        indices_quad = np.divide(indices, 90)
        unique_quad = np.unique(indices_quad)
        unique_quad += 1
        quad_str = ' '.join(map(str, unique_quad))
        print("Warning! Object detected")
        print("Object detected in quadrant " + quad_str)

        pub.publish(quad_str)
        r.sleep()


# quad = 0 means no object detected, quad 1,2,3,4 means object is in respective quadrant
pub = rospy.Publisher('/scan/quadrant', String, queue_size=1)


rospy.init_node('laser_listener', anonymous=True)
r = rospy.Rate(15)  # 10hz
while not rospy.is_shutdown():

    rospy.Subscriber('/laser/scan', LaserScan, callback)
    rospy.spin()
