#!/usr/bin/env python
# coding=utf-8

# Node ROS regulation

# S'abonne aux commande demandées et extrait les données demandées pour les republier sous forme de float64

import rospy
import numpy as np
from std_msgs.msg import Float64, Int16
import tf.transformations


def getprop(obj, string):
    """
    Par exemple 'position.x'
    :param string:
    :return:
    """
    tab = string.split('.')

    curr_val = obj
    for str in tab:
        curr_val = getattr(curr_val, str)

    return curr_val


class Data_extractor:

    def __init__(self, wanted_data):
        self.dataTab = [0.0, 0.0, 0.0]

    def callback(self, msg):

        # Recuperation des donnes
        if not (wanted_data == ""):
            data = getprop(msg, wanted_data)
        else:
            data = msg

        # Operation
        if operation is not None:
            fct = getattr(self, 'get_' + operation)
            data = fct(data)

        # Gain
        if gain is not None:
            print "data :", data
            datalen = len(dir(data))-len(dir(Float64()))
            if datalen == 2:
                data.x = data.x*float(gain)
                data.y = data.y*float(gain)
                data.z = data.z*float(gain)
            else:
                data = data*float(gain)
        
        print "datagain :", data

        # Lissage
        if smooth is not None:
            self.dataTab.append(data)
            if len(self.dataTab) > int(smooth):
                self.dataTab.remove(self.dataTab[0])
            data = np.mean(self.dataTab)
        
        # Translation
        if import_str_out_msg_class == 'Float64':
            data = Float64(data)
        if import_str_out_msg_class == 'Int16':
            data = Int16(data)

        print "datapub :", data
        pub.publish(data)

    def get_quaternion(self, quaternion):
        quaternion = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

    def get_heading(self, vector):
        x = vector.x
        y = vector.y
        return np.arctan2(y, x)

    def get_norm(self, vector):
        x = vector.x
        y = vector.y
        return np.sqrt(x**2+y**2)

    def get_int(self, data):
        return int(data.data)


if __name__ == '__main__':
    rospy.init_node('data_extractor')

    # OUTPUT
    import_str_out_msg_module = rospy.get_param('~out_data_format_mod', 'std_msgs.msg')
    import_str_out_msg_class = rospy.get_param('~out_data_format_cla', 'Float64')
    Msg_out_module = __import__(import_str_out_msg_module, fromlist=[import_str_out_msg_class])
    Msg_out_class = getattr(Msg_out_module, import_str_out_msg_class)    
    print 'import result Msg_out :', Msg_out_class
    out_topic = rospy.get_param('~out_topic')
    pub = rospy.Publisher(out_topic, Msg_out_class, queue_size=1)

    wanted_data = rospy.get_param('~wanted_data')
    operation = rospy.get_param('~operation', None)
    gain = rospy.get_param('~gain', None)
    smooth = rospy.get_param('~smooth', None)
    ext = Data_extractor(wanted_data)

    # INPUT (in_data_format_mod, in_data_format_mod, in_topic)
    import_str_in_msg_module = rospy.get_param('~in_data_format_mod')
    import_str_in_msg_class = rospy.get_param('~in_data_format_cla')
    Msg_in_module = __import__(import_str_in_msg_module, fromlist=[import_str_in_msg_class])
    Msg_in_class = getattr(Msg_in_module, import_str_in_msg_class)
    print 'import result Msg_in :', Msg_in_class
    in_topic = rospy.get_param('~in_topic')
    sub = rospy.Subscriber(in_topic, Msg_in_class, ext.callback)

    
    rospy.spin()

