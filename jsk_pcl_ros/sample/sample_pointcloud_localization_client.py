#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from std_srvs.srv import EmptyRequest


if __name__ == "__main__":
    rospy.init_node("sample_pointcloud_localization_client")
    rospy.wait_for_service('~localize')
    localize_client = rospy.ServiceProxy('~localize', Empty)
    r = rospy.Rate(rospy.get_param('~request_rate', 1.0))

    while not rospy.is_shutdown():
        req = EmptyRequest()
        res = localize_client(req)

        try:
            r.sleep()
        except rospy.ROSTimeMovedBackwardsException:
            pass
