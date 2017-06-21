#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Wrench
from std_msgs.msg import Int16
import numpy as np

class Motor():


    def __init__(self, namespace, x=0.0, y=0.0, tension=16):  
        self.namespace = namespace 

        self.x = x
        self.y = y
        self.tension = tension
        self.maxThrustTab = {16:11.7,12:7.29}
        self.maxThrust = self.maxThrustTab[self.tension]

        self.cmdThr = 0
        self.cmdPos = 0

        self.pos = 0
        
        self.pubCmdThr = rospy.Publisher(namespace+'/cmd_thr', Int16, queue_size=1)  # -100 to 100
        self.pubCmdPos = rospy.Publisher(namespace+'/cmd_pos', Int16, queue_size=1)  # -180 to 180

        self.subPos = rospy.Subscriber(namespace+'/servoPos', Int16, self.updatePosServo)


    def updatePosServo(self, msg):
        self.pos = msg.data # feedback


    def computeCmd(self, F):
        self.cmdThr = np.trunc(np.linalg.norm(F)/self.maxThrust*100)
        if np.isnan(self.cmdThr):
            rospy.logwarn("cmdThr is NaN")
            self.cmdThr = 0.0
        self.cmdPos = np.trunc(np.arctan2(F[1],F[0])*180/np.pi)
        if np.isnan(self.cmdPos):
            rospy.logwarn("cmdPos is NaN")
            self.cmdPos = 0.0


    def publishCmd(self):
        # print 'Publishing cmd for',self.namespace,': pos:',self.cmdPos,' and thr:',self.cmdThr
        self.pubCmdPos.publish(self.cmdPos)
        self.pubCmdThr.publish(self.cmdThr)



class Buggy_command():


    def __init__(self):

        self.subWrench = rospy.Subscriber('cmd_wrench', Wrench, self.updateWrench)

        self.motorAv = Motor('avant',x=1.2)
        self.motorAr = Motor('arriere',x=-1.2)

        self.cmdWrenchTorZ = 0.0
        self.cmdWrenchForX = 0.0
        self.cmdWrenchForY = 0.0


    def updateWrench(self, msg):
        # print 'Received ',msg
        self.cmdWrenchTorZ = msg.torque.z
        self.cmdWrenchForX = msg.force.x
        self.cmdWrenchForY = msg.force.y

        Fa, Fb = self.computeForces(np.array([self.motorAv.x,0.0,0.0]),
                                    np.array([self.motorAr.x,0.0,0.0]),
                                    np.array([0.0,0.0,0.0]),
                                    np.array([0.0,0.0,self.cmdWrenchTorZ]),
                                    np.array([self.cmdWrenchForX,self.cmdWrenchForY,0.0]))
        print "Fa :",Fa
        print "Fb :",Fb

        self.motorAv.computeCmd(Fa)
        self.motorAr.computeCmd(Fb)
        self.motorAv.publishCmd()
        self.motorAr.publishCmd()
        rospy.loginfo('\n[Pav:%d, Tav:%d,\n Par:%d, Tar:%d]',self.motorAv.cmdPos,self.motorAv.cmdThr,self.motorAr.cmdPos,self.motorAr.cmdThr)

    def process(self):
        pass # Inutile ici mais au moins c'est la si jamais on a besoin d'action asynchrones


    def computeForces(self, A, B, I, M, R):
        '''
        Vecteurs de dimension 3
        A : Position du moteur avant par rapport au robot
        B : Position du moteur arrière par rapport au robot
        I : Position du centre d'inertie du robot
        R : Résultante des forces au point I
        M : Moment appliqué par les forces sur le robot au point I
        '''

        xa = A[0]; ya = A[1]
        xb = B[0]; yb = B[1]
        xi = I[0]; yi = I[1]
        xr = R[0]; yr = R[1]
        zm = M[2]

        # xfa = ( xa*xr**2/2 + xa*yr**2/2 - xb*xr**2/2 + xb*yr**2/2 - xi*yr**2 - xr*yb*yr + xr*yi*yr - yr*zm)/(xa*xr - xb*xr + ya*yr - yb*yr)
        # xfb = ( xa*xr**2/2 - xa*yr**2/2 - xb*xr**2/2 - xb*yr**2/2 + xi*yr**2 + xr*ya*yr - xr*yi*yr + yr*zm)/(xa*xr - xb*xr + ya*yr - yb*yr)
        # yfa = (-xb*xr*yr + xi*xr*yr + xr**2*ya/2 + xr**2*yb/2 - xr**2*yi + xr*zm + ya*yr**2/2 - yb*yr**2/2)/(xa*xr - xb*xr + ya*yr - yb*yr)
        # yfb = ( xa*xr*yr - xi*xr*yr - xr**2*ya/2 - xr**2*yb/2 + xr**2*yi - xr*zm + ya*yr**2/2 - yb*yr**2/2)/(xa*xr - xb*xr + ya*yr - yb*yr)

        # Probleme de singularité au dénominateur en cas xr=0 et ya-yb=0
        # si xr = 0 alors yfa = yr/2.0 et yfb = yr/2.0
        # si yr = 0 & xr=0 alors xfa = 0 & xfb = 0
        
        if xr==0:
            xfa = 0.0
            xfb = 0.0
        else:
            xfa = ( (xa*xr**2 + xa*yr**2 - xb*xr**2 + xb*yr**2)/2.0 - xi*yr**2 - xr*yb*yr + xr*yi*yr - yr*zm) / ((xa-xb)*xr + (ya-yb)*yr)
            xfb = ( (xa*xr**2 - xa*yr**2 - xb*xr**2 - xb*yr**2)/2.0 + xi*yr**2 + xr*ya*yr - xr*yi*yr + yr*zm) / ((xa-xb)*xr + (ya-yb)*yr)

        if xr==0:
            yfa = (yr + (xa-xi)*zm)/2.0
            yfb = (yr + (xb-xi)*zm)/2.0
        else:
            yfa = (-xb*xr*yr + xi*xr*yr + (xr**2*ya + xr**2*yb)/2.0 - xr**2*yi + xr*zm + ((ya-yb)*yr**2)/2.0) / ((xa-xb)*xr + (ya-yb)*yr)
            yfb = ( xa*xr*yr - xi*xr*yr - (xr**2*ya - xr**2*yb)/2.0 + xr**2*yi - xr*zm + ((ya-yb)*yr**2)/2.0) / ((xa-xb)*xr + (ya-yb)*yr)
        
        return np.array([xfa,yfa,0]), np.array([xfb,yfb,0])
 

if __name__ == '__main__':
    rospy.init_node('cmd_buggy')
    r = rospy.Rate(50)
    kayak = Kayak_command()
    rospy.spin()
    #while not rospy.is_shutdown():
    #    kayak.process()
    #    r.sleep()

