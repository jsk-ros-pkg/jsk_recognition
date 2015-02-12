#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge

rospy.init_node("image_publisher")

rate = rospy.Rate(rospy.get_param("rate", 1))
file_name = rospy.get_param("~file_name", "image.png")
pub = rospy.Publisher("~output", Image)
bridge = cv_bridge.CvBridge()
while not rospy.is_shutdown():
    try:
        image = cv2.imread(file_name)
        image_message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        image_message.header.stamp = rospy.Time.now()
        pub.publish(image_message)
    except Exception, e:
        rospy.loginfo("cannot read the image at %s" % file_name)
        rospy.loginfo(e.message)
    rate.sleep()
    
