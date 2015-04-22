#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

#ROS imports
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
#Python imports
import numpy as np

#redefinitions and global variables
sin = np.sin
cos = np.cos
pi = np.pi

# Robotic Agent Initial Conditions

x = 0.0 
y = 0.0 
h = 0.0
v = 0.5 #Linear Velocity
w = -0.05 #Angular Velocity (Omega)

def start():
    """Simulates turtlebot movement
    and publishes new state continuously"""

    global x, y, h, v, w
    global pub

    #ROS setup
    pub = rospy.Publisher('virtual_agent/pose', PoseStamped, queue_size = 100)
    rospy.init_node('Virtual_Agent')
    hz = 200
    r = rospy.Rate(hz)

    #ROS main loop
    while not rospy.is_shutdown():

        dt = 1/float(hz)
        #simulate turtlebot dynamics
        dx = v*cos(h)
        dy = v*sin(h)
        dh = w
        x = x + dt * dx
        y = y + dt * dy
        h = h + dt * dh

        #ROS Pose format with Quaternions Orientation:
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = '/virtual_base_link'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0
        ps.pose.orientation.x = 0
        ps.pose.orientation.y = 0
        ps.pose.orientation.z = sin(h/2.0)
        ps.pose.orientation.w = cos(h/2.0)

        #Publish message to topic
        pub.publish(ps)
        r.sleep()

if __name__ == '__main__':
    
    try:
        start()
    except rospy.ROSInterruptException:
        pass

