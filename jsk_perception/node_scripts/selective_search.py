#!/usr/bin/env python

import rospy

import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import RectArray, Rect
from sensor_msgs.msg import Image
import dynamic_reconfigure.server
from jsk_perception.cfg import SelectiveSearchConfig
import dlib
import cv2

class SelectiveSearch(ConnectionBasedTransport):
    def __init__(self):
        super(SelectiveSearch, self).__init__()
        dynamic_reconfigure.server.Server(
            SelectiveSearchConfig, self._cb_dyn_reconfig)
        self.pub_ = self.advertise('~output', RectArray, queue_size=10)
        self.debug_pub_ = self.advertise('~debug', Image, queue_size=1)

    def subscribe(self):
        self.sub_ = rospy.Subscriber('~input', Image, self._apply, queue_size=1)

    def unsubscribe(self):
        self.sub_.unregister()
    def _cb_dyn_reconfig(self, config, level):
        self.min_size = config.min_size
        self.max_size = config.max_size
        return config
    def _apply(self, img_msg):
        bridge = cv_bridge.CvBridge()
        img_bgr = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        rects = []
        dlib.find_candidate_object_locations(img_rgb, rects,
                                             min_size=self.min_size)
        ros_rect_array = RectArray()
        ros_rect_array.header = img_msg.header
        for d in rects:
            if (d.right() - d.left()) * (d.bottom() - d.top()) > self.max_size:
                continue
            cv2.rectangle(img_bgr, (d.left(), d.top()), (d.right(), d.bottom()), (255, 0, 0), 3)
            ros_rect_array.rects.append(Rect(x=d.left(), y=d.top(), width=d.right() - d.left(), height=d.bottom() - d.top()))
        imgmsg = bridge.cv2_to_imgmsg(img_bgr, encoding='bgr8')
        self.debug_pub_.publish(imgmsg)
        self.pub_.publish(ros_rect_array)


if __name__ == '__main__':
    rospy.init_node('selective_search')
    selective_search = SelectiveSearch()
    rospy.spin()
