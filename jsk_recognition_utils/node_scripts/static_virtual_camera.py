#!/usr/bin/env python

import cv2
import numpy as np
try:
    from scipy.misc import face
    img = face()[:, :, ::-1]
except ImportError:
    from scipy.misc import lena
    img = cv2.cvtColor(lena().astype(np.uint8), cv2.COLOR_GRAY2BGR)

import cv_bridge
import rospy
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import CameraInfo


def timer_cb(event):
    imgmsg.header.stamp = rospy.Time.now()
    info_msg.header.stamp = imgmsg.header.stamp
    if pub_img.get_num_connections() > 0:
        pub_img.publish(imgmsg)
    if pub_img_compressed.get_num_connections() > 0:
        pub_img_compressed.publish(compmsg)
    pub_info.publish(info_msg)


if __name__ == '__main__':
    rospy.init_node('static_virtual_camera')

    # get image_trasport before ConnectionBasedTransport subscribes ~input
    transport_hint = rospy.get_param('~image_transport', 'raw')
    rospy.loginfo("Using transport {}".format(transport_hint))

    pub_img = rospy.Publisher('~image_color', Image, queue_size=1)
    pub_img_compressed = rospy.Publisher('~image_color/compressed', CompressedImage, queue_size=1)
    pub_info = rospy.Publisher('~camera_info', CameraInfo, queue_size=1)

    bridge = cv_bridge.CvBridge()
    imgmsg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    imgmsg.header.frame_id = 'static_virtual_camera'
    compmsg = CompressedImage()
    compmsg.header = imgmsg.header
    compmsg.format = "jpeg"
    compmsg.data = np.array(cv2.imencode('.jpg', cv2.cvtColor(img, cv2.COLOR_BGR2RGB))[1]).tostring()

    info_msg = CameraInfo()
    info_msg.header.frame_id = imgmsg.header.frame_id
    height, width = img.shape[:2]
    info_msg.height = height
    info_msg.width = width

    rospy.Timer(rospy.Duration(0.1), timer_cb)
    rospy.spin()
