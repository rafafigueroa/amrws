#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

def od_callback(robot_od):
    x = robot_od.pose.pose.position.x
    r = 2.0
    k = 0.5
    v = k * (r - x)

    robot_tw = Twist()
    robot_tw.linear.x = v
    pub.publish(robot_tw)

def control():
    rospy.init_node('amr_control')
    rospy.Subscriber('/odom', Odometry, od_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
