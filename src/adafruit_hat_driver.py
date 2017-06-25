#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32

from Adafruit_MotorHAT import Adafruit_PWM_Servo_Driver as pwmDriver
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

class MotorBoard:

    def __init__(self):
        self.mh = Adafruit_MotorHAT(addr=0x60)
        atexit.register(turnOffMotors)

    def servo_cmd(self, pin, angle):
        # TESTS, pour une frequence de 50Hz, et 0 en ON, on a des valeurs correcte pour les servo de 101 a 560
        # 16*6 = 96, 16*35=560
        # Milieu a 330 (230 de chaque cote)

        print 'angle :', angle
        print 'pin :', pin

        cmd = angle/180.0*230+330

        self.mh.setPWMFreq(50)
        self.mh.setPWM(int(pin, 0, int(cmd)))


    # recommended for auto-disabling motors on shutdown!
    def turnOffMotors(self):
        self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

    def cb_servo1(self, msg):
        self.servo_cmd(14, msg.data)

    def cb_servo2(self, msg):
        self.servo_cmd(15, msg.data)

    def cb_motor_gauche(self, msg):
        self.mh.getMotor(1).setSpeed(msg.data)
        self.mh.getMotor(3).setSpeed(msg.data)

    def cb_motor_droit(self, msg):
        self.mh.getMotor(2).setSpeed(msg.data)
        self.mh.getMotor(4).setSpeed(msg.data)


if __name__ == '__main__':
    rospy.init_node("motorBoard_driver")

    motor = MotorBoard()

    rospy.Subscriber('cmd_position_servo1', Float32, motor.cb_servo1)
    rospy.Subscriber('cmd_position_servo2', Float32, motor.cb_servo2)

    rospy.Subscriber('gauche/cmd_thr', Int16, motor.cb_motor_gauche)
    rospy.Subscriber('droite/cmd_tghr', Int16, motor.cb_motor_droit)

    rospy.spin()
