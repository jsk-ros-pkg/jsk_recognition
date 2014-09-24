#!/usr/bin/env python

import rospy
from resized_image_client import ResizedImageClient
from std_msgs.msg import Float64


class TrafficMode():
    stop = 0
    error = 1
    narrow = 2
    normal = 3
    wide = 4

class RospingRIC(ResizedImageClient):
    def __init__(self, node, ping_node, x=0.1, y=0.1, mps=1.0):
        ResizedImageClient.__init__(self, node, x, y, mps)
        self.sub_ping = rospy.Subscriber(ping_node, Float64, self.pingCB)
        self.old_mode = TrafficMode.stop
        self.mode = TrafficMode.narrow

    def pingCB(self, data):
        delay = data.data

        # sample
        # if delay > 0:
        # self.msg_per_second = 10.0
        # self.resize_scale_x = 1.0/(1.0 + delay)
        # self.resize_scale_y = 1.0/(1.0 + delay)

        if delay < 0:
            self.mode = TrafficMode.error
        elif delay < 10.0:
            self.mode = TrafficMode.wide
        elif delay < 500.0:
            self.mode = TrafficMode.normal
        else:
            self.mode = TrafficMode.narrow

        if self.old_mode != self.mode:
            if self.mode == TrafficMode.error:
                self.msg_per_second = 10.0
                self.resize_scale_x = 0.1
                self.resize_scale_y = 0.1
            elif self.mode == TrafficMode.narrow:
                self.msg_per_second = 1.0
                self.resize_scale_x = 0.1
                self.resize_scale_y = 0.1
            elif self.mode == TrafficMode.normal:
                self.msg_per_second = 1.0
                self.resize_scale_x = 0.3
                self.resize_scale_y = 0.3
            elif self.mode == TrafficMode.wide:
                self.msg_per_second = 1.0
                self.resize_scale_x = 1.0
                self.resize_scale_y = 1.0

            self.update()
            self.old_mode = self.mode

if __name__ == '__main__':
    rospy.init_node('rosping_resized_image_client')
    image_node = rospy.get_param('image_node', '/head_resized')
    delay_node = rospy.get_param('delay_node', '/ping/delay')
    ric = RospingRIC(image_node, delay_node)
    rospy.spin()
    while True:
        ric.update()
        time.sleep(1)



