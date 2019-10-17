#!/usr/bin/env python

import os.path as osp
import unittest

import rospy
import rostest


class TestLINEMODTrainer(unittest.TestCase):

    def test_linemod_trainer(self):
        linemod_path = osp.expanduser(rospy.get_param('~linemod_path'))
        pcd_path = osp.expanduser(rospy.get_param('~pcd_path'))
        yaml_path = osp.expanduser(rospy.get_param('~yaml_path'))
        timeout = rospy.Duration(rospy.get_param('~timeout', 30.0))
        rospy.sleep(2.0)  # Wait a moment until /clock is published.
        start = rospy.Time.now()
        while (rospy.Time.now() - start < timeout and
               (not osp.exists(linemod_path) or
                not osp.exists(pcd_path) or
                not osp.exists(yaml_path))):
            rospy.sleep(1.0)
        self.assertTrue(osp.isfile(linemod_path))
        self.assertTrue(osp.isfile(pcd_path))
        self.assertTrue(osp.isfile(yaml_path))


if __name__ == '__main__':
    rospy.init_node('test_linemod_trainer')
    rostest.rosrun(
        'jsk_pcl_ros', 'test_linemod_trainer', TestLINEMODTrainer)
