#!/usr/bin/env python

import rospy
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import Rect, RectArray
from image_view2.msg import ImageMarker2
from jsk_recognition_utils import color_category10
from geometry_msgs.msg import Point
class RectArrayToImageMarker(ConnectionBasedTransport):
    def __init__(self):
        super(RectArrayToImageMarker, self).__init__()
        self.pub = self.advertise("~output", ImageMarker2, queue_size=1)
        
    def subscribe(self):
        self.sub = rospy.Subscriber("~input", RectArray, self.convert)
        
    def unsubscribe(self):
        self.sub.unregister()
        
    def convert(self, msg):
        marker = ImageMarker2()
        marker.header = msg.header
        marker.type = ImageMarker2.LINE_LIST
        for rect, rect_i in zip(msg.rects, range(len(msg.rects))):
            points = [(rect.x, rect.y),
                      (rect.x, rect.y + rect.height),
                      (rect.x + rect.width, rect.y + rect.height),
                      (rect.x + rect.width, rect.y)]
            for i, j in ((0, 1), (1, 2), (2, 3), (3, 0)):
                marker.points.append(Point(x=points[i][0], y=points[i][1]))
                marker.points.append(Point(x=points[j][0], y=points[j][1]))
                marker.outline_colors.append(color_category10(rect_i))
                marker.outline_colors.append(color_category10(rect_i))
        self.pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('rect_array_to_image_marker')
    rect_array_to_image_marker = RectArrayToImageMarker()
    rospy.spin()
