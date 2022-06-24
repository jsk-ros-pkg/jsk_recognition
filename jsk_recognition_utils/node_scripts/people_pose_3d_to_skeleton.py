#!/usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function

from jsk_topic_tools import ConnectionBasedTransport
import rospy
from jsk_recognition_msgs.msg import HumanSkeleton
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_recognition_msgs.msg import Segment

class PeoplePose3DtoSkeleton(ConnectionBasedTransport):
    
    def __init__(self):
        super(self.__class__, self).__init__()
        self.limb_sequence = rospy.get_param('~limb_sequence')
        self.index2limbname = rospy.get_param('~index2limbname')
        self.pub = self.advertise('~output', HumanSkeletonArray, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', PeoplePoseArray, self._convert)

    def unsubscribe(self):
        self.sub.unregister()

    def _convert(self, msg):
        skeleton_array_msg = HumanSkeletonArray(header=msg.header)
        
        for people_pose in msg.poses:
            skeleton_msg = HumanSkeleton()
            for i, conn in enumerate(self.limb_sequence):
                j1_name = self.index2limbname[conn[0] - 1]
                j2_name = self.index2limbname[conn[1] - 1]
                if j1_name not in people_pose.limb_names \
                        or j2_name not in people_pose.limb_names:
                    continue
                j1_index = people_pose.limb_names.index(j1_name)
                j2_index = people_pose.limb_names.index(j2_name)
                bone_name = '{}->{}'.format(j1_name, j2_name)
                bone = Segment(
                    start_point=people_pose.poses[j1_index].position,
                    end_point=people_pose.poses[j2_index].position)
                skeleton_msg.bones.append(bone)
                skeleton_msg.bone_names.append(bone_name)
            skeleton_array_msg.skeletons.append(skeleton_msg)

        self.pub.publish(skeleton_array_msg)

if __name__ == '__main__':
    rospy.init_node('people_pose_3d_to_skeleton')
    PeoplePose3DtoSkeleton()
    rospy.spin()
