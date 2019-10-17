#!/usr/bin/env python

import unittest

import rospy
import rostest
from std_srvs.srv import Empty

from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.srv import SaveMesh
from jsk_recognition_msgs.srv import SaveMeshResponse


class SampleBoundingBoxPublisher(object):

    def __init__(self):
        self.pub = rospy.Publisher('~bbox_output', BoundingBox, queue_size=1)
        rospy.sleep(0.1)
        rospy.Timer(rospy.Duration(0.1), self._cb)

    def _cb(self, event):
        self.box = BoundingBox()
        self.box.header.stamp = event.current_real
        self.box.header.frame_id = 'dummy_frame'
        self.box.pose.position.x = 1.0
        self.box.pose.position.y = 2.0
        self.box.pose.position.z = 3.0
        self.box.pose.orientation.x = 4.0
        self.box.pose.orientation.y = 5.0
        self.box.pose.orientation.z = 6.0
        self.box.pose.orientation.w = 7.0
        self.box.dimensions.x = 8.0
        self.box.dimensions.y = 9.0
        self.box.dimensions.z = 10.0
        self.box.value = 11.0
        self.box.label = 12
        self.pub.publish(self.box)


class SampleSaveMeshRequestCaller(object):

    def __init__(self):
        rospy.Timer(rospy.Duration(1.0), self._cb)

    def _cb(self, event):
        rospy.wait_for_service('~request')
        req_caller = rospy.ServiceProxy('~request', Empty)
        self.res = req_caller()


class TestSaveMeshServer(unittest.TestCase):

    def test_save_mesh_server(self):
        self.bbox_publisher = SampleBoundingBoxPublisher()
        self.req_caller = SampleSaveMeshRequestCaller()
        self.cb_tested = False
        self.srv_server = rospy.Service('~save_mesh', SaveMesh, self._cb)
        while self.cb_tested is False:
            rospy.sleep(0.1)
        self.assertTrue(self.cb_tested is True)

    def _cb(self, req):
        # Check if ground_frame_id parameter and that of request are the same.
        assert rospy.get_param('~ground_frame_id') == req.ground_frame_id

        # Check if published bbox and that of request are the same.
        assert self.bbox_publisher.box == req.box

        self.cb_tested = True
        return SaveMeshResponse(True)


if __name__ == '__main__':
    rospy.init_node('test_save_mesh_server')
    rostest.rosrun(
        'jsk_recognition_msgs', 'test_save_mesh_server', TestSaveMeshServer)
