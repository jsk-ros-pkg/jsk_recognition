#!/usr/bin/env python
# -*- coding: utf-8 -*-

# copied from image_view2
# workaround until https://github.com/jsk-ros-pkg/jsk_common/pull/1774 is merged and released
#

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image
import cv_bridge
from image_view2.msg import MouseEvent


def main():
    pub_plus = rospy.Publisher('~plus_rect_event', MouseEvent, queue_size=1)
    pub_minus = rospy.Publisher('~minus_rect_event', MouseEvent, queue_size=1)

    width = int(rospy.get_param('~image_width'))
    height = int(rospy.get_param('~image_height'))
    plus_events = [
        MouseEvent(type=3, x=int(width/4), y=int(height/4), width=width, height=height),
        MouseEvent(type=4, x=int(width/2), y=int(height/2), width=width, height=height),
        MouseEvent(type=2, x=int(3*width/4), y=int(3*height/4), width=width, height=height),
    ]
    minus_events = [
        MouseEvent(type=3, x=int(3*width/4), y=int(3*height/4), width=width, height=height),
        MouseEvent(type=4, x=int(width/2), y=int(height/2), width=width, height=height),
        MouseEvent(type=2, x=int(width/4), y=int(height/4), width=width, height=height),
    ]
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for e in plus_events:
            e.header.stamp = rospy.get_rostime()
            pub_plus.publish(e)
            rate.sleep()
        for e in minus_events:
            e.header.stamp = rospy.get_rostime()
            pub_minus.publish(e)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('publish_mouse_event')
    main()
