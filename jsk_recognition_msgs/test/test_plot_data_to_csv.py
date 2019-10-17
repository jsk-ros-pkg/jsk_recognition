#!/usr/bin/env python

import os.path as osp
import unittest

import rospy
import rostest


class TestPlotDataToCsv(unittest.TestCase):

    def test_plot_data_to_csv(self):
        csv_path = osp.expanduser(rospy.get_param('~csv_path'))
        timeout = rospy.Duration(rospy.get_param('~timeout', 5.0))
        start = rospy.Time.now()
        while rospy.Time.now() - start < timeout and not osp.exists(csv_path):
            rospy.sleep(0.1)
        self.assertTrue(osp.isfile(csv_path))


if __name__ == '__main__':
    rospy.init_node('test_plot_data_to_csv')
    rostest.rosrun(
        'jsk_recognition_msgs', 'test_plot_data_to_csv', TestPlotDataToCsv)
