#!/usr/bin/env python
import rospy

PKG='jsk_pcl_ros'

import imp
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

from sensor_msgs.msg import PointCloud2
from jsk_pcl_ros.srv import SetPointCloud2

def cloud_cb(cloud):

    try:
        setCloud = rospy.ServiceProxy('particle_filter_tracker/renew_model', SetPointCloud2)
        setCloud(cloud)
        rospy.loginfo("Success renew_tracking service");
    except rospy.ServiceException, e:
            rospy.loginfo("Failed to call renew_tracking service")

if __name__ == "__main__":
    rospy.init_node('send_renew_model', anonymous=True)
    rospy.wait_for_service('particle_filter_tracker/renew_model')
    rospy.Subscriber("selected_pointcloud", PointCloud2, cloud_cb)
    rospy.spin()
