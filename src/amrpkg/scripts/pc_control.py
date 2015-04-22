#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import numpy as np


def pc_callback(robot_pc):
    pc = robot_pc.data
    print pc

    if False:
        r = 1.0
        k = 0.5
        v = - k * (r - x)

        robot_tw = Twist()
        robot_tw.linear.x = v
        pub.publish(robot_tw)

def control():
    rospy.init_node('amr_control')
    rospy.Subscriber('/camera/depth/points', PointCloud2,
                     pc_callback)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('/mobile_base/commands/velocity', 
                          Twist, queue_size=10)

    if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
