#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

def sc_callback(robot_sc):
    scan_ranges = robot_sc.ranges
    #print 'scan ranges len', len(scan_ranges)

    x = scan_ranges[320]
    print 'x:', x

    if not np.isnan(x):
        r = 1.0
        k = 0.5
        v = - k * (r - x)

        robot_tw = Twist()
        robot_tw.linear.x = v
        pub.publish(robot_tw)

def control():
    rospy.init_node('amr_control')
    rospy.Subscriber('/scan', LaserScan, sc_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass


