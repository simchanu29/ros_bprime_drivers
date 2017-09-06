#!/usr/bin/env python
# coding=utf-8

# Node ROS pose_fusion

# S'abonne aux capteurs qui donnent la position (ici imu et gps)
# puis publie une pose et un twist

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion
import numpy as np
import Utility.geometry as geom
import tf.transformations


def Msg(parent):
    """
    Dynamic msg class to implement function fill_header
    :param parent:
    :return: message with fill_header
    """
    class Msg(parent):
        def __init__(self):
            super(Msg, self).__init__()
            # print self

        def fill_header(msg, frame_id=''):
            msg.header.stamp.secs = rospy.get_rostime().secs
            msg.header.stamp.nsecs = rospy.get_rostime().nsecs
            msg.header.frame_id = frame_id
            return msg
    return Msg()


class Pose_fusion():
    def __init__(self):
        # Origin
        self.lat_origin = 60.0  # x
        self.lon_origin = 0.0  # y

        # Publish
        self.pose = Msg(PoseStamped)
        self.pose_pub = rospy.Publisher('pose_est', PoseStamped, queue_size=1)
        self.twist = Msg(TwistStamped)
        self.twist_pub = rospy.Publisher('twist_est', TwistStamped, queue_size=1)

        # Subscribe
        self.imu = Msg(Imu)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.updateImu)
        self.gpsfix = Msg(NavSatFix)
        self.gpsfix_sub = rospy.Subscriber('gps/fix', NavSatFix, self.updateGpsFix)
        self.gpsvel = Msg(TwistStamped)
        self.gpsvel_sub = rospy.Subscriber('gps/vel', TwistStamped, self.updateGpsVel)
        self.gpsfix.latitude = 60.0*1852*60

    def updateImu(self, msg):
        self.imu = msg
        # self.process()

    def updateGpsFix(self, msg):
        self.gpsfix = msg
        # self.process()

    def updateGpsVel(self, msg):
        self.gpsvel = msg
        # self.process()

    def process(self):
        # Fusion cap
        # A basse vitesse (<2m/s) on prend le cap de l'imu
        # A haute vitesse (>2m/s) on prend le cap du gps
        # Entre les deux on fait une transition linéaire
        quaternion = (self.imu.orientation.x,
                      self.imu.orientation.y,
                      self.imu.orientation.z,
                      self.imu.orientation.w)
        euler_imu = tf.transformations.euler_from_quaternion(quaternion)
        head_imu = euler_imu[2]  # rad
        head_gps = np.arctan2(self.gpsvel.twist.linear.y, self.gpsvel.twist.linear.x)  # rad
        linvel2d = (self.gpsvel.twist.linear.y ** 2.0 + self.gpsvel.twist.linear.x ** 2.0) ** 0.5

        transitorystate_lb = 1.5
        transitorystate_ub = 2.5
        coeff_choice = np.max([np.min([linvel2d * (transitorystate_ub - transitorystate_lb)
                                      - transitorystate_lb,
                                      1.0]),
                              0.0])
        head = head_gps * coeff_choice + head_imu * (1 - coeff_choice)

        self.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(euler_imu[0], euler_imu[1], head))

        # Local conversion of longlat coordinates
        # Convention NED, donc le z est vers le bas et le x vers le nord
        lon = self.gpsfix.longitude
        lat = self.gpsfix.latitude
        CONVERSION_FACTOR_GPS = 1852  # en metres/min d'angle
        self.pose.pose.position.x = (lat-self.lat_origin)*CONVERSION_FACTOR_GPS*60.0  # lat->x
        self.pose.pose.position.y = (lon-self.lon_origin)*CONVERSION_FACTOR_GPS*60.0*np.cos(self.lat_origin/180.0*np.pi)  # lon->y
        self.pose.pose.position.z = 0.0

        # Gestion du twist
        # On change la vitesse linéaire pour la remettre dans un repere local
        # gpsvel est ici un twiststamped
        gpsvel = self.gpsvel
        gpsvel_lin = gpsvel.twist.linear
        vel_lin = geom.rotation(np.array([gpsvel_lin.x, gpsvel_lin.y]), -head)
        gpsvel.twist.linear.x = vel_lin[0]
        gpsvel.twist.linear.y = vel_lin[1]

        self.twist = gpsvel
        self.twist.twist.angular = self.imu.angular_velocity


        # Header
        self.pose.header.stamp.secs = rospy.get_rostime().secs
        self.pose.header.stamp.nsecs = rospy.get_rostime().nsecs
        self.twist.header.stamp.secs = rospy.get_rostime().secs
        self.twist.header.stamp.nsecs = rospy.get_rostime().nsecs

        # Publication
        self.pose_pub.publish(self.pose)
        self.twist_pub.publish(self.twist)


if __name__ == '__main__':
    rospy.init_node('pose_fusion')
    r = rospy.Rate(100)
    node = Pose_fusion()
    # rospy.spin()
    while not rospy.is_shutdown():
        node.process()
        r.sleep()
