#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import TrackArray
from cv_bridge import CvBridge
from jsk_recognition_utils.panorama_utils import visualize_tracks


class DrawTracks(object):

    def __init__(self):

        self.bridge = CvBridge()
        self.pub_viz = rospy.Publisher('~output', Image, queue_size=1)
        self.subscribe()

    def subscribe(self):

        queue_size = rospy.get_param('~queue_size', 100)
        sub_image = message_filters.Subscriber('~input', Image)
        sub_tracks = message_filters.Subscriber('~input/tracks', TrackArray)
        self.subs = [sub_image, sub_tracks]
        if rospy.get_param('~approximate_sync', False):
            slop = rospy.get_param('~slop', 0.1)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            self.sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        self.sync.registerCallback(self.callback)

    def callback(self, img_msg, tracks_msg):

        if self.pub_viz.get_num_connections() > 0:
            input_frame = self.bridge.imgmsg_to_cv2(
                img_msg, desired_encoding='bgr8')
            visualized_frame = visualize_tracks(input_frame, tracks_msg)
            msg = self.bridge.cv2_to_imgmsg(visualized_frame, "bgr8")
            msg.header = img_msg.header
            self.pub_viz.publish(msg)


if __name__ == '__main__':
    rospy.init_node('draw_tracks')
    node = DrawTracks()
    rospy.spin()
