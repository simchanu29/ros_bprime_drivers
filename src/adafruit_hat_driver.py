import rospy
from std_msgs.msg import Int16

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
    rospy.Subscriber('cmd_positionServo1', Int16, cb_servo1)
    rospy.Subscriber('cmd_positionServo2', Int16, cb_servo2)
    rospy.Subscriber('cmd_mot_av_g', Int16, cb_motor_av_gauche)
    rospy.Subscriber('cmd_mot_av_d', Int16, cb_motor_av_droit)
    rospy.Subscriber('cmd_mot_ar_g', Int16, cb_motor_ar_gauche)
    rospy.Subscriber('cmd_mot_ar_d', Int16, cb_motor_ar_droit)
    rospy.spin()