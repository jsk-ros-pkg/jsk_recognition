#!/usr/bin/env python

import rospy

from jsk_topic_tools import ConnectionBasedTransport

from jsk_recognition_msgs.msg import PanoramaInfo


class ResizePanoramaInfo(ConnectionBasedTransport):

    def __init__(self):
        super(ResizePanoramaInfo, self).__init__()
        self.scale_height = float(rospy.get_param('~scale_height', 1.0))
        self.scale_width = float(rospy.get_param('~scale_width', 1.0))
        self.pub = self.advertise('~output', PanoramaInfo, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input', PanoramaInfo, self._sub_cb, queue_size=1)

    def unsubscribe(self):
        self.sub.unregister()

    def _sub_cb(self, msg):
        msg.image_height = int(msg.image_height * self.scale_height)
        msg.image_width = int(msg.image_width * self.scale_width)
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('resize_panorama_info')
    app = ResizePanoramaInfo()
    rospy.spin()
