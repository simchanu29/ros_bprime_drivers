#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32

def cb_servo1(msg):
    pass

def cb_servo2(msg):
    pass

def cb_motor_av_gauche(msg):
    pass

def cb_motor_av_droit(msg):
    pass

def cb_motor_ar_gauche(msg):
    pass

def cb_motor_ar_droit(msg):
    pass

if __name__ == '__main__':
    rospy.init_node("motorBoard_driver")
    rospy.Subscriber('cmd_position_servo1', Float32, cb_servo1)
    rospy.Subscriber('cmd_position_servo2', Float32, cb_servo2)
    rospy.Subscriber('gauche/cmd_thr', Int16, cb_motor_av_gauche)
    rospy.Subscriber('droite/cmd_thr', Int16, cb_motor_av_droit)
    rospy.spin()