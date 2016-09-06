#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import rospy
from threading import Event
from unittest import TestCase

import dynamic_reconfigure.client
from jsk_recognition_msgs.msg import PolygonArray, ModelCoefficientsArray
from geometry_msgs.msg import PolygonStamped
from pcl_msgs.msg import ModelCoefficients

TEST_NODE = "/polygon_array_unwrapper"

class TestPolygonArrayUnwrapper(TestCase):
    def setUp(self):
        self.client = dynamic_reconfigure.client.Client(TEST_NODE, timeout=30)
        self.pub_polygon = rospy.Publisher(TEST_NODE + "/input_polygons",
                                           PolygonArray, queue_size=1)
        self.pub_coefficients = rospy.Publisher(TEST_NODE + "/input_coefficients",
                                                ModelCoefficientsArray, queue_size=1)
        self.sub_polygon = None
        self.sub_coefficients = None
        self.msg_wait_timeout = 10
        self.polygon_msg = None
        self.polygon_msg_callback_event = Event()
        self.coeff_msg = None
        self.coeff_msg_callback_event = Event()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publishPolygons)
    def tearDown(self):
        self.unsubscribe()
        self.timer.shutdown()
        self.polygon_msg_callback_event.clear()
        self.coeff_msg_callback_event.clear()
    def subscribe(self):
        self.sub_polygon = rospy.Subscriber(TEST_NODE + "/output_polygon",
                                            PolygonStamped,
                                            self.polygonMsgCallback)
        self.sub_coefficients = rospy.Subscriber(TEST_NODE + "/output_coefficients",
                                                 ModelCoefficients,
                                                 self.coeffMsgCallback)
    def unsubscribe(self):
        self.sub_polygon.unregister()
        self.sub_coefficients.unregister()
    def polygonMsgCallback(self, msg):
        self.polygon_msg = msg
        self.polygon_msg_callback_event.set()
    def coeffMsgCallback(self, msg):
        self.coeff_msg = msg
        self.coeff_msg_callback_event.set()
    def publishPolygons(self, event=None):
        pmsg = PolygonArray()
        pmsg.header.stamp = rospy.Time.now()
        cmsg = ModelCoefficientsArray()
        cmsg.header.stamp = pmsg.header.stamp
        for i in range(10):
            pmsg.polygons.append(PolygonStamped())
            pmsg.polygons[i].header.stamp = pmsg.header.stamp
            pmsg.polygons[i].header.frame_id = str(i)
            cmsg.coefficients.append(ModelCoefficients())
            cmsg.coefficients[i].header.stamp = cmsg.header.stamp
            cmsg.coefficients[i].header.frame_id = str(i)
        pmsg.likelihood = [1.0,2.0,3.0,4.0,5.0,4.0,3.0,2.0,1.0,0.0]
        self.pub_polygon.publish(pmsg)
        self.pub_coefficients.publish(cmsg)
    def waitMsgs(self):
        self.polygon_msg_callback_event.wait(self.msg_wait_timeout)
        self.coeff_msg_callback_event.wait(self.msg_wait_timeout)
    def testUnwrap(self):
        self.client.update_configuration({
            "plane_index": 0,
            "use_likelihood": False
        })
        self.subscribe()
        self.waitMsgs()

        self.assertTrue(self.polygon_msg is not None)
        self.assertTrue(self.coeff_msg is not None)
        self.assertEqual(self.polygon_msg.header.frame_id, str(0))
        self.assertEqual(self.coeff_msg.header.frame_id, str(0))
    def testUnwrapIndex(self):
        self.client.update_configuration({
            "plane_index": 7,
            "use_likelihood": False
        })
        self.subscribe()
        self.waitMsgs()

        self.assertTrue(self.polygon_msg is not None)
        self.assertTrue(self.coeff_msg is not None)
        self.assertEqual(self.polygon_msg.header.frame_id, str(7))
        self.assertEqual(self.coeff_msg.header.frame_id, str(7))
    def testUnwrapLikelihood(self):
        self.client.update_configuration({
            "plane_index": 2,
            "use_likelihood": True
        })
        self.subscribe()
        self.waitMsgs()

        self.assertTrue(self.polygon_msg is not None)
        self.assertTrue(self.coeff_msg is not None)
        self.assertEqual(self.polygon_msg.header.frame_id, str(4))
        self.assertEqual(self.coeff_msg.header.frame_id, str(4))

if __name__ == '__main__':
    import rostest
    rospy.init_node("polygon_array_unwrapper_test")
    rostest.rosrun("jsk_pcl_ros_utils", "polygon_array_unwrapper_test", TestPolygonArrayUnwrapper)
