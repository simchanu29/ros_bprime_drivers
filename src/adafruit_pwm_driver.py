#!/usr/bin/env python

import rospy
from ros_bprime_drivers.msg import PwmCmd
from Adafruit_PWM_Servo_Driver.Adafruit_PWM_Servo_Driver import PWM

class PWMBoard:

    def __init__(self, pin_dic):
        self.pwm = PWM(0x40)
        self.pwm.setPWMFreq(50)
        self.pins = pin_dic

    def cb_pwm(self, msg):
        # TESTS, pour une frequence de 50Hz, et 0 en ON, on a des valeurs correcte pour les servo de 101 a 560
        # 16*6 = 96, 16*35=560
        # Milieu a 330 (230 de chaque cote)

        print 'pin :', msg.pin
        print 'cmd :', msg.command

        type = self.pins[msg.pin]
        if type=='servo':
            cmd = msg.cmd / 180.0 * 500 + 1500
        elif type=='motor':
            cmd = msg.cmd*5 + 1500
        else:
            cmd = 1500
        self.setPWM(msg.pin, cmd)

    def setPWM(self, pin, cmd):
        cmd = (cmd-1000)/1000*409
        self.pwm.setPWM(pin, 0, cmd) #max 4095, 1 cycle = 4095. Donc 409 max (20ms -> 2ms)

if __name__ == '__main__':
    rospy.init_node("motorBoard_driver")

    pwmboard = PWMBoard(rospy.get_param('~pin_dic'))
    rospy.Subscriber('pwm_cmd', Pwm_cmd, pwmboard.cb_pwm)

    rospy.spin()
