#!/usr/bin/env python

from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray
from jsk_recognition_msgs.msg import RectArray
from jsk_topic_tools import ConnectionBasedTransport
import rospy


class RectArrayToPolygonArray(ConnectionBasedTransport):

    def __init__(self):
        super(RectArrayToPolygonArray, self).__init__()
        self.pub = self.advertise('~output', PolygonArray, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', RectArray, self._convert)

    def unsubscribe(self):
        self.sub.unregister()

    def _convert(self, msg):
        polys_msg = PolygonArray()
        polys_msg.header = msg.header
        for rect in msg.rects:
            poly_msg = PolygonStamped()
            poly_msg.header = msg.header
            x0, y0 = float(rect.x), float(rect.y)
            x1, y1 = float(rect.x + rect.width), float(rect.y + rect.height)
            pt0 = Point32(x=x0, y=y0)
            pt1 = Point32(x=x0, y=y1)
            pt2 = Point32(x=x1, y=y1)
            pt3 = Point32(x=x1, y=y0)
            poly_msg.polygon.points.append(pt0)
            poly_msg.polygon.points.append(pt1)
            poly_msg.polygon.points.append(pt2)
            poly_msg.polygon.points.append(pt3)
            polys_msg.polygons.append(poly_msg)
        self.pub.publish(polys_msg)


if __name__ == '__main__':
    rospy.init_node('rect_array_to_polygon_array')
    RectArrayToPolygonArray()
    rospy.spin()
