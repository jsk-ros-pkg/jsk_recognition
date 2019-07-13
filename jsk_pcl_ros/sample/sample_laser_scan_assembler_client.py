#!/usr/bin/env python

from __future__ import division

from laser_assembler.srv import AssembleScans2
import rospy
from sensor_msgs.msg import PointCloud2


if __name__ == '__main__':
    rospy.init_node('sample_laser_scan_assembler_client')
    pub = rospy.Publisher('~output', PointCloud2, queue_size=1)
    rospy.wait_for_service('~assemble_scans2')
    r = rospy.Rate(rospy.get_param('~rate', 1.0))
    while not rospy.is_shutdown():
        try:
            srv_caller = rospy.ServiceProxy('~assemble_scans2', AssembleScans2)
            res = srv_caller(rospy.Time(0), rospy.Time.now())
            pub.publish(res.cloud)
        except rospy.ServiceException as e:
            rospy.logwarn('Service call failed:\n{}'.format(e))

        try:
            r.sleep()
        except rospy.ROSTimeMovedBackwardsException:
            pass
