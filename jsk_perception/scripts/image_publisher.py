#!/usr/bin/env python

import cv2

import rospy
import cv_bridge
from sensor_msgs.msg import Image


rospy.init_node("image_publisher")

rate = rospy.Rate(rospy.get_param("rate", 1))
file_name = rospy.get_param("~file_name", "image.png")
pub = rospy.Publisher("~output", Image, queue_size=1)
bridge = cv_bridge.CvBridge()
while not rospy.is_shutdown():
    try:
        image = cv2.imread(file_name)
        image_message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        image_message.header.stamp = rospy.Time.now()
        pub.publish(image_message)
    except IOError, e:
        rospy.loginfo("cannot read the image at %s" % file_name)
        rospy.loginfo(e.message)
    rate.sleep()

