#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np

def minAngle(ang):
    return np.arctan2(np.sin(ang), np.cos(ang))

def orientation_to_yaw(orientation):
    quat_list = [0, 0, 0, 0]
    quat_list[0] = orientation.x
    quat_list[1] = orientation.y
    quat_list[2] = orientation.z
    quat_list[3] = orientation.w
    (roll, pitch, yaw) = euler_from_quaternion(quat_list)
    return yaw

class SimModel(object):
    """Provides a consistent usage of simulation models"""
    def __init__(self, control):
        self.control = control

class SimMain(object):
    
    def __init__(self,  model, hz = 50.0):
        self.state = [None]*6
        self.model = model

        rospy.init_node('amr_control')

        self.rate = rospy.Rate(hz)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, 
                                   queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.state_callback)
        rospy.Subscriber('/virtual_agent/pose', PoseStamped, 
                         self.virtual_state_callback)
        
        print("simulation initialized")
        #TODO: Wait for topic

    def state_callback(self, robot_od):
        x = robot_od.pose.pose.position.x
        y = robot_od.pose.pose.position.y
        h = orientation_to_yaw(robot_od.pose.pose.orientation)

        #print 'state', x, y , h

        self.state[0] = x
        self.state[1] = y
        self.state[2] = h

    def virtual_state_callback(self, robot_ps):
        xr = robot_ps.pose.position.x
        yr = robot_ps.pose.position.y
        hr = orientation_to_yaw(robot_ps.pose.orientation)

        self.state[3] = xr
        self.state[4] = yr
        self.state[5] = hr
        #print 'virtual', xr, yr, hr

    def run(self):

        print("simulation running")
        while not rospy.is_shutdown():
            if (self.state[0] is not None) and \
               (self.state[3] is not None):

                u = self.model.control(self.state)
                v = u[0]
                w = u[1]

                robot_tw = Twist()
                robot_tw.linear.x = v
                robot_tw.angular.z = w

                self.pub.publish(robot_tw)

            self.rate.sleep()

#TODO: Make general, currently copy/paste from virtual
vr = 0.5
wr = -0.05

def control_virtual_linear(X):
        x = X[0]
        y = X[1]
        h = X[2]
        xr = X[3]
        yr = X[4]
        hr = X[5]

        # (s+2*xi*alpha)*(s**2 + 2*xi*alpha*s + alpha**2)
        # poles at -p1r +- p1i*j and -p2
        # (s+p2)*(s+p1r+p1i)*(s+p1r-p1i)
        # (s+p2)*(s**2 + 2*p1r*s + p1r**2 + p1i**2)
        # 2*xi*alpha = p2
        # 2*xi*alpha = 2*p1r
        # alpha**2 = p1r**2 + p1i**2
        # for p1r = 1, p1i = 1, p2 = 2*p1r = 2 
        
        # Linear
        p1r = 1 
        p1i = 0.3
        p2 = 2*p1r
        alpha = np.sqrt(p1r**2+p1i**2)
        xi = p2/float(2*alpha)

        b = (alpha**2 - wr**2)/float(vr**2)

        k1 = 2*xi*alpha
        k2 = b * np.abs(vr)
        k3 = k1
        
        ex = xr - x
        ey = yr - y
        eh = minAngle(hr - h)

        e1 = ex*np.cos(h) + ey*np.sin(h)
        e2 = -ex*np.sin(h) + ey*np.cos(h)
        e3 = eh

        u1 = -k1 * e1
        u2 = -k2 * np.sign(vr) * e2 - k3 * e3
        
        v = vr * np.cos(e3) - u1
        w = wr - u2
        Erms = np.sqrt(e1**2 + e2**2 + e3**2)
        print 'Erms:', Erms, 'v', v, 'w', w

        return [v, w]

def control_virtual_nonlinear(X):
        x = X[0]
        y = X[1]
        h = X[2]
        xr = X[3]
        yr = X[4]
        hr = X[5]

        # (s+2*xi*alpha)*(s**2 + 2*xi*alpha*s + alpha**2)
        # poles at -p1r +- p1i*j and -p2
        # (s+p2)*(s+p1r+p1i)*(s+p1r-p1i)
        # (s+p2)*(s**2 + 2*p1r*s + p1r**2 + p1i**2)
        # 2*xi*alpha = p2
        # 2*xi*alpha = 2*p1r
        # alpha**2 = p1r**2 + p1i**2
        # for p1r = 1, p1i = 1, p2 = 2*p1r = 2 

        # Nonlinear 
        p1r = 0.5 
        p1i = 0.1
        p2 = 2*p1r
        alpha = np.sqrt(p1r**2+p1i**2)
        xi = p2/float(2*alpha)

        b = (alpha**2 - wr**2)/float(vr**2)

        k1 = 2*xi*np.sqrt(wr**2 + b*vr**2)
        k2 = b * np.abs(vr)
        k3 = k1
        k4 = b
        
        ex = xr - x
        ey = yr - y
        eh = minAngle(hr - h)

        e1 = ex*np.cos(h) + ey*np.sin(h)
        e2 = -ex*np.sin(h) + ey*np.cos(h)
        e3 = eh

        u1 = -k1 * e1
        u2 = -k4 * vr * np.sin(e3)/(e3+0.001) * e2 - k3 * e3
        
        v = vr * np.cos(e3) - u1
        w = wr - u2
        Erms = np.sqrt(e1**2 + e2**2 + e3**2)
        print 'Erms:', Erms, 'v', v, 'w', w

        return [v, w]


if __name__ == '__main__':
    sim_model = SimModel(control = control_virtual_nonlinear)
    sim = SimMain(sim_model)

    try:
        sim.run()
    except rospy.ROSInterruptException:
        pass


   





