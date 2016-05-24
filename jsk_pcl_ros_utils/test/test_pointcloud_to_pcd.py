#!/usr/bin/env python

import unittest
import rospy
import roslib.packages
import glob
import time

class PointCloudToPCD():
    def __init__(self):
        self.test_data_path = roslib.packages.get_pkg_dir('jsk_pcl_ros_utils') + '/test_data/sample_pcd_*.pcd'
        self.time_limit = 360 

    def check_pcd_generate(self):
        cnt = 0
        while cnt <= self.time_limit:
            pcd_files = glob.glob(self.test_data_path)
            if pcd_files:
                return True
            cnt += 1
            time.sleep(1.0)
        return False

class TestPointCloudToPCD(unittest.TestCase):
    def test_point_clout_to_pcd(self):
        pcd = PointCloudToPCD()
        self.assertTrue(pcd.check_pcd_generate())

if __name__ == "__main__":
    import rostest
    rospy.init_node("pointcloud_to_pcd_test_py")
    rostest.rosrun("jsk_pcl_ros_utils", "test_pointcloud_to_pcd", TestPointCloudToPCD)
