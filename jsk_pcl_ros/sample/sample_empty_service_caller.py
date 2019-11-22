#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty


if __name__ == '__main__':
    rospy.init_node('sample_empty_service_caller')
    rospy.wait_for_service('~service')
    r = rospy.Rate(rospy.get_param('~rate', 1.0))
    while not rospy.is_shutdown():
        try:
            srv_caller = rospy.ServiceProxy('~service', Empty)
            res = srv_caller()
        except rospy.ServiceException as e:
            rospy.logwarn('Service call failed:\n{}'.format(e))

        try:
            r.sleep()
        except rospy.ROSTimeMovedBackwardsException:
            pass
