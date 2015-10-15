#!/usr/bin/env python
import os

import cv2

import rospy
import cv_bridge
import dynamic_reconfigure.server

from jsk_perception.cfg import ImagePublisherConfig
from sensor_msgs.msg import Image, CameraInfo


def _cb_dyn_reconfig(config, level):
    global file_name
    file_name = config['file_name']
    config['file_name'] = os.path.abspath(file_name)
    return config


rospy.init_node("image_publisher")

dynamic_reconfigure.server.Server(ImagePublisherConfig, _cb_dyn_reconfig)
rate = rospy.Rate(rospy.get_param("rate", 1))
if_publish_info = rospy.get_param("~publish_info", True)
pub = rospy.Publisher("~output", Image, queue_size=1)
if if_publish_info:
    pub_info = rospy.Publisher("~output/camera_info", CameraInfo, queue_size=1)
bridge = cv_bridge.CvBridge()
while not rospy.is_shutdown():
    try:
        now = rospy.Time.now()
        image = cv2.imread(file_name)
        if image is None:
            raise IOError('image value is None')
        image_message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        image_message.header.stamp = now
        image_message.header.frame_id = "camera"
        info = CameraInfo()
        info.header.stamp = now
        info.header.frame_id = "camera"
        info.width = image_message.width
        info.height = image_message.height
        if if_publish_info:
            pub_info.publish(info)
        pub.publish(image_message)
    except IOError, e:
        rospy.loginfo("cannot read the image at %s" % file_name)
        rospy.loginfo(e.message)
    except AttributeError, e:
        rospy.logerr("Did you set properly ~file_name param ?")
        rospy.loginfo(e.message)
    rate.sleep()

