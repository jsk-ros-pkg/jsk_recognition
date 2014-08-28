#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client
import time

class ResizedImageClient():
    def __init__(self, node, x=0.1, y=0.1, mps=1.0):
        self.client = dynamic_reconfigure.client.Client(node)
        self.resize_scale_x = x
        self.resize_scale_y = y
        self.msg_par_second = mps

    def update(self):
        params = { 'resize_scale_x' : self.resize_scale_x, 'resize_scale_y' : self.resize_scale_y, 'msg_par_second' : self.msg_par_second}
        rospy.loginfo("update config \nresized_scale_x : %s \nresized_scale_y : %s \nmsg_par_second : %s", self.resize_scale_x, self.resize_scale_y, self.msg_par_second)
        config = self.client.update_configuration(params)


if __name__ == '__main__':
    rospy.init_node('resized_image_client_sample')
    ric = ResizedImageClient('/head_resized')
    while True:
        ric.update()
        time.sleep(1)
    
