#!/usr/bin/env python
# -*- coding: utf-8 -*-

import dynamic_reconfigure.client
import rospy
from jsk_recognition_msgs.msg import Int32Stamped


def cb(msg):
    cfg = client.get_configuration(timeout=None)
    tol_orig = cfg['tolerance']
    delta = tol_orig * reconfig_eps
    if msg.data == expected_n_cluster:
        print('Expected/Actual n_cluster: {0}, Tolerance: {1}'
              .format(msg.data, tol_orig))
        return
    elif msg.data > expected_n_cluster:
        cfg['tolerance'] += delta
    else:
        cfg['tolerance'] -= delta
    print('''\
Expected n_cluster: {0}
Actual   n_cluster: {1}
tolerance: {2} -> {3}
'''.format(expected_n_cluster, msg.data, tol_orig, cfg['tolerance']))
    client.update_configuration(cfg)


if __name__ == '__main__':
    rospy.init_node('k_dynamic_euclid_clutering')
    reconfig_eps = rospy.get_param('~reconfig_eps', 0.2)
    node_name = rospy.get_param('~node')
    expected_n_cluster = rospy.get_param('~k_cluster')
    client = dynamic_reconfigure.client.Client(node_name)
    sub_ncluster = rospy.Subscriber(
        '{node}/cluster_num'.format(node=node_name), Int32Stamped, cb)
    rospy.spin()
