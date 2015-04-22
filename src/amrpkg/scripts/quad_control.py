#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from gazebo_msgs.srv import GetModelState
from hector_uav_msgs.msg import MotorPWM
import numpy as np

def control():
    rospy.init_node('amr_control')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub_pwm = rospy.Publisher('/motor_pwm', MotorPWM, queue_size=10)
    rate = rospy.Rate(100)

    rospy.wait_for_service('/gazebo/get_model_state/')
    print 'service ready'

    while not rospy.is_shutdown():

        get_model_state = \
        rospy.ServiceProxy('/gazebo/get_model_state/',
                           GetModelState)
        quad_state = get_model_state('quadrotor', 'world')
        z = quad_state.pose.position.z
        print 'z:', z

        simple_controller = True

        if simple_controller:
            robot_tw = Twist()
            robot_tw.linear.z = (1.4-z)*0.3
            pub.publish(robot_tw)
        else:
            # TODO: Fix direct control, not currently working well
            motor_pwm = MotorPWM()
            k = 60
            pwm = 40+int(k*(1.4-z))
            print pwm
            motor_pwm.pwm = [pwm, pwm, pwm, pwm]
            pub_pwm.publish(motor_pwm)



        rate.sleep()


if __name__ == '__main__':
   try:
        control()
    except rospy.ROSInterruptException:
        pass


