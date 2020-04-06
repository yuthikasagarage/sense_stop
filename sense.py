#!/usr/bin/env python2
import rospy
import time
import sys
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray


def callback(data):
    ranges = data.ranges
    rangesarray = np.array(ranges, dtype=np.float)
    sample_indices = np.where(
        np.logical_and(rangesarray <= 3, rangesarray != np.float('inf')))
    quad_str = '0'
    if sample_indices[0].size != 0:
        indices = sample_indices[0]
        quad = np.array_split(rangesarray, 8)
        min_angle_q1 = 1000
        min_angle_q2 = 1000
        min_angle_q3 = 1000
        min_angle_q4 = 1000

        quad1 = np.concatenate((quad[1], quad[2]))

        minquad1 = float(np.min(quad1))
        if minquad1 < 5:
            min_angle_quad1 = np.where(quad1 == np.amin(quad1, axis=0))
            min_angle_q1 = int(min_angle_quad1[0])

        else:
            minquad1 = 1000
            min_angle_quad1 = 1000

        quad2 = np.concatenate((quad[3], quad[4]))
        minquad2 = float(np.min(quad2))
        if minquad2 < 5:
            min_angle_quad2 = np.where(quad2 == np.amin(quad2, axis=0))
            min_angle_q2 = int(min_angle_quad2[0])

        else:
            minquad2 = 1000
            min_angle_quad2 = 1000

        quad3 = np.concatenate((quad[5], quad[6]))
        minquad3 = float(np.min(quad3))
        if minquad3 < 5:
            min_angle_quad3 = np.where(quad3 == np.amin(quad3, axis=0))
            min_angle_q3 = int(min_angle_quad3[0])
        else:
            minquad3 = 1000
            min_angle_quad3 = 1000

        quad4 = np.concatenate((quad[7], quad[0]))
        minquad4 = float(np.min(quad4))

        if minquad4 < 5:
            min_angle_quad4 = np.where(quad4 == np.amin(quad4, axis=0))
            min_angle_q4 = int(min_angle_quad4[0])

        else:
            minquad4 = 1000
            min_angle_quad4 = 1000

        indices_quad = np.divide(indices, 90)
        unique_quad = np.unique(indices_quad)
        unique_quad += 1
        quad_str = ' '.join(map(str, unique_quad))
        print("Object detected in quadrant " + quad_str)

        hello_float = Float32MultiArray()
        hello_float.data = [
            minquad1, min_angle_q1, minquad2, min_angle_q2, minquad3,
            min_angle_q3, minquad4, min_angle_q4
        ]

        print(minquad1, min_angle_q1, minquad2, min_angle_q2, minquad3,
              min_angle_q3, minquad4, min_angle_q4)
        pub.publish(quad_str)
        pub1.publish(hello_float)
        r.sleep()


# quad = 0 means no object detected, quad 1,2,3,4 means object is in respective quadrant
pub = rospy.Publisher('/scan/quadrant', String, queue_size=1)

pub1 = rospy.Publisher('/scan/min', Float32MultiArray, queue_size=1)

rospy.init_node('laser_listener', anonymous=True)
r = rospy.Rate(15)  # 10hz
while not rospy.is_shutdown():

    rospy.Subscriber('/laser/scan', LaserScan, callback)
    rospy.spin()
