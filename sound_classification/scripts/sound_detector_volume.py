#!/usr/bin/env python

from cv_bridge import CvBridge
from sound_classification.msg import InSound
from sensor_msgs.msg import Image
import rospy
from topic_tools import LazyTransport


class SoundDetectorVolume(LazyTransport):
    """
    Detect whether the robot is in sound or not by sound volume.

    Robot is detected as 'in_sound' when
    the average power of pixels is more then 'power_per_pixel_threshold'
    """

    def __init__(self):
        super(self.__class__, self).__init__()
        # ROS
        self.threshold = rospy.get_param('~power_per_pixel_threshold', 0)
        self.pub = self.advertise('~in_sound', InSound, queue_size=1)
        self.cv_bridge = CvBridge()

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Image, self._cb)

    def unsubscribe(self):
        self.sub.unregister()

    def _cb(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg)
        pub_msg = InSound()
        pub_msg.header = msg.header
        power_per_pixel = img.sum() / img.size
        rospy.logdebug('power_per_pixel: {}, threshold: {}'.format(
            power_per_pixel, self.threshold))
        if power_per_pixel > self.threshold:
            rospy.logdebug('### In sound ###')
            pub_msg.in_sound = True
        else:
            rospy.logdebug('No sound')
            pub_msg.in_sound = False
        self.pub.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node('sound_detector_volume')
    a = SoundDetectorVolume()
    rospy.spin()
