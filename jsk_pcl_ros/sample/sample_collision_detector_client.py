#!/usr/bin/env python

from __future__ import division

from geometry_msgs.msg import PoseStamped
from jsk_recognition_msgs.srv import CheckCollision
from jsk_recognition_msgs.srv import CheckCollisionRequest
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class SampleCollisionDetectorClient(object):

    def __init__(self):
        super(self.__class__, self).__init__()

        self.joint_states = None
        self.root_link_pose = PoseStamped()
        self.root_link_pose.pose.orientation.w = 1

        self.req_caller = rospy.ServiceProxy(
            '~check_collision', CheckCollision)
        self.pub = rospy.Publisher('~output', Bool, queue_size=1)
        self.sub = rospy.Subscriber(
            '~joint_states', JointState, self._joint_cb, queue_size=1)

        rate = rospy.get_param('~request_rate', 1.0)
        rospy.Timer(rospy.Duration(1 / rate), self._timer_cb)

    def _joint_cb(self, msg):
        self.joint_states = msg
        self.root_link_pose.header = msg.header

    def _timer_cb(self, event):
        if self.joint_states is None:
            return

        req = CheckCollisionRequest()
        req.joint = self.joint_states
        req.pose = self.root_link_pose
        res = self.req_caller(req)
        self.pub.publish(data=res.result)


if __name__ == '__main__':
    rospy.init_node('sample_collision_detector_client')
    app = SampleCollisionDetectorClient()
    rospy.spin()
