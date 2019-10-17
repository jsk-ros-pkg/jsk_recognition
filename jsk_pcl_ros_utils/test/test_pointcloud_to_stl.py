#!/usr/bin/env python

import os.path as osp
import unittest

import rospy
import rostest


class TestPointCloudToSTL(unittest.TestCase):

    def test_pointcloud_to_stl(self):
        stl_path = osp.expanduser(rospy.get_param('~stl_path'))
        timeout = rospy.Duration(rospy.get_param('timeout', 30.0))
        start = rospy.Time.now()
        while rospy.Time.now() - start < timeout and not osp.exists(stl_path):
            rospy.sleep(1.0)
        self.assertTrue(osp.isfile(stl_path))


if __name__ == '__main__':
    rospy.init_node('test_pointcloud_to_stl')
    rostest.rosrun(
        'jsk_pcl_ros_utils', 'test_pointcloud_to_stl', TestPointCloudToSTL)
