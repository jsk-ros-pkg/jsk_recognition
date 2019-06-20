#!/usr/bin/env python

import os.path as osp
import unittest

import rospy
import rostest


class TestPlotDataToCsv(unittest.TestCase):

    def test_plot_data_to_csv(self):
        csv_path = rospy.get_param('~csv_path')
        self.assertTrue(osp.isfile(osp.expanduser(csv_path)))


if __name__ == '__main__':
    rospy.init_node('test_plot_data_to_csv')
    rostest.rosrun(
        'jsk_recognition_msgs', 'test_plot_data_to_csv', TestPlotDataToCsv)
