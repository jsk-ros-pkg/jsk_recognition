#!/usr/bin/env python

import time
import sys
import math
import rospy
import tf
import json
import numpy as np
from numpy.linalg import inv
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from jsk_perception.cfg import TimeStampConfig

class timestamp_changer:

    def init(self):
        rospy.init_node('timestamp', anonymous=True)

        self.__sub_topic_name = rospy.get_param("~sub_topic_name", "/odom_input")
        self.__pub_topic_name = rospy.get_param("~pub_topic_name", "/odom_output")

        self.__sub_odometry = rospy.Subscriber(self.__sub_topic_name, Odometry, self.__odom_callback)
        self.__pub_odometry = rospy.Publisher(self.__pub_topic_name, Odometry, queue_size = 10)

        self.__time_diff  = 0.0

        self.__cfg_srv = Server(TimeStampConfig, self.__cfg_callback)

        rospy.spin()

    def __cfg_callback(self, config, level):
        if config.enable_flag == True:
            self.__time_diff = config.time_diff
            rospy.loginfo("change time diff to %f", self.__time_diff);
            print self.__time_diff
        return config

    def __odom_callback(self, msg):
        t = msg.header.stamp.to_sec()
        print self.__time_diff
        msg.header.stamp = rospy.Time.from_sec(t + self.__time_diff)
        self.__pub_odometry.publish(msg)

if __name__ == '__main__':
    try:
        TimeStampChanger = timestamp_changer()
        TimeStampChanger.init()
    except rospy.ROSInterruptException:
        pass


