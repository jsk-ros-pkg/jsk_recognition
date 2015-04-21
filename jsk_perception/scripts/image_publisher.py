#!/usr/bin/env python

import cv2

import rospy
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo


rospy.init_node("image_publisher")

rate = rospy.Rate(rospy.get_param("rate", 1))
file_name = rospy.get_param("~file_name", "image.png")
pub = rospy.Publisher("~output", Image, queue_size=1)
pub_info = rospy.Publisher("~output/camera_info", CameraInfo, queue_size=1)
bridge = cv_bridge.CvBridge()
while not rospy.is_shutdown():
    try:
        now = rospy.Time.now()
        image = cv2.imread(file_name)
        image_message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        image_message.header.stamp = now
        image_message.header.frame_id = "camera"
        info = CameraInfo()
        info.header.stamp = now
        info.header.frame_id = "camera"
        info.width = image_message.width
        info.height = image_message.height
        pub_info.publish(info)
        pub.publish(image_message)
    except IOError, e:
        rospy.loginfo("cannot read the image at %s" % file_name)
        rospy.loginfo(e.message)
    except AttributeError, e:
        rospy.logerr("Did you set properly ~file_name param ?")
        rospy.loginfo(e.message)
        exit()
    rate.sleep()

