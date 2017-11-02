#!/usr/bin/env python
# coding=utf-8

# Node ROS simu_thruster

# S'abonne aux commande et position moteur pour traduire ça en
# force et en moment sur le système


import rospy
import numpy as np
from std_msgs.msg import Int16
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import TwistStamped


class T200BlueRobotics():
    def __init__(self, x, y, tension):
        self.tension = tension
        self.x = x  # coordonnee le long de l'USV
        self.y = y  # perpendiculairement au cap de l'USV
        self.maxPower = 300
        self.maxThrustInfo = {16: 11.23, 12: 7.29}
        self.maxThrust = self.maxThrustInfo[tension]

        print(self.x, self.y, self.tension, self.maxThrust)


class ClassicMotor():
    def __init__(self, x, y, tension):
        self.tension = tension
        self.x = x  # coordonnee le long du vehicule
        self.y = y  # perpendiculairement au cap du vehicule
        self.maxPower = 50
        self.motoReduction = 1
        self.wheelDiameter = 0.15  # m
        # self.maxThrustInfo = {16: 11.23, 12: 7.29}
        self.maxThrust = float(tension) / 5.0 * self.wheelDiameter * self.motoReduction * self.maxPower

        print(self.x, self.y, self.tension, self.maxThrust)


class SimDynMot():
    def __init__(self):
        self.sub_cmd_thrust = rospy.Subscriber('cmd_thr', Int16, self.update_cmd_thrust)
        self.cmd_thrust = 0
        self.sub_pos_thrust = rospy.Subscriber('cmd_pos', Int16, self.update_cmd_pos)
        self.cmd_pos = 0
        self.sub_vel = rospy.Subscriber('simu/twist_real', TwistStamped, self.up)

        self.pubWrenchThruster = rospy.Publisher('simu_force', Wrench, queue_size=1)  # -100 to 100
        self.wrenchThruster = Wrench()

        self.thruster = ClassicMotor(rospy.get_param('x'),
                                     rospy.get_param('y'),
                                     rospy.get_param('tension'))

    def update_cmd_thrust(self, msg):
        self.cmd_thrust = msg.data

    def update_cmd_pos(self, msg):
        self.cmd_pos = msg.data

    def process(self):
        self.wrenchThruster.force.x = self.thruster.maxThrust * self.cmd_thrust / 100.0 * np.cos(
            self.cmd_pos / 180.0 * np.pi)
        self.wrenchThruster.force.y = self.thruster.maxThrust * self.cmd_thrust / 100.0 * np.sin(
            self.cmd_pos / 180.0 * np.pi)
        self.wrenchThruster.force.z = 0

        # la force s'applique en (x,y) avec un angle a par rapport au cap du vehicule.
        # x est la coordonnee le long du vehicule
        self.wrenchThruster.torque.x = 0
        self.wrenchThruster.torque.y = 0
        self.wrenchThruster.torque.z = self.wrenchThruster.force.y * self.thruster.x

        self.pubWrenchThruster.publish(self.wrenchThruster)


if __name__ == '__main__':
    rospy.init_node('simu_thruster')
    r = rospy.Rate(10)
    simu = SimDynMot()
    while not rospy.is_shutdown():
        simu.process()
        r.sleep()
