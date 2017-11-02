#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64, Bool

class Interpreter:
    """
    This node translates Twist messages via joystick to vector3 by publishing the linear component of the Twist message
    and a heading commande.
    There is a security : a timeout. If no command has been sent then the command is [0,0,0].
    """
    def __init__(self):
        self.timeout = 1.5
        self.time = 0.0
        self.cmd_joy = Twist()

    def update_joy(self, msg):
        self.cmd_joy = msg
        self.cmd_joy.linear.x = self.cmd_joy.linear.x*0.2
        self.cmd_joy.linear.y = self.cmd_joy.linear.y*0.2
        self.cmd_joy.linear.z = self.cmd_joy.linear.z*0.2
        pub_cmdvel.publish(self.cmd_joy.linear)
        pub_cmdhead.publish(msg.angular.z*2.0)
        self.time = 0.0

    def process(self):
        print self.time
        if self.time<1.5:
            pub_secureteleop.publish(True)
        else:
            pub_secureteleop.publish(False)

if __name__ == '__main__':
    rospy.init_node('joy_interpreter')
    rate = 20.0
    r = rospy.Rate(rate)
    interpreter = Interpreter()

    pub_cmdvel = rospy.Publisher('/cmd_vel', Vector3, queue_size=1)
    pub_cmdhead = rospy.Publisher('/cmd_head', Float64, queue_size=1)
    pub_secureteleop = rospy.Publisher('/secure_teleop', Bool, queue_size=1)

    sub_joy = rospy.Subscriber('/cmd_joy', Twist, interpreter.update_joy)
    
    while not rospy.is_shutdown():
       interpreter.time += 1/rate
       interpreter.process()
       r.sleep()
