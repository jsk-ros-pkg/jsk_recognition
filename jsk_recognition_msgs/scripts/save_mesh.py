#!/usr/bin/env python

import argparse

import rospy
from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.srv import SaveMesh
from jsk_recognition_msgs.srv import SaveMeshRequest


class SaveMeshApp(object):

    def __init__(self, ground_frame_id='map'):
        self.ground_frame_id = ground_frame_id
        self.sub_bbox = rospy.Subscriber('~input/bbox', BoundingBox, self._cb)
        self.srv_client = rospy.ServiceProxy('~save_mesh', SaveMesh)
        self.bbox_msg = None

    def _cb(self, bbox_msg):
        self.bbox_msg = bbox_msg

    def mainloop(self):
        while app.bbox_msg is None:
            rospy.sleep(0.1)
        req = SaveMeshRequest()
        req.box = app.bbox_msg
        req.ground_frame_id = self.ground_frame_id
        app.srv_client.call(req)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--ground-frame-id', default='map')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('save_mesh')
    app = SaveMeshApp(ground_frame_id=args.ground_frame_id)
    app.mainloop()
