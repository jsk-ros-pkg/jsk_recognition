#!/usr/bin/env python

# it depends on jsk_rviz_plugins

import rospy
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import Float32
from threading import Lock
from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface
from jsk_recognition_msgs.msg import TrackerStatus
g_lock = Lock()
g_angle_error_msg = None
g_distance_error_msg = None
def angle_error_callback(msg):
    global g_angle_error_msg
    with g_lock:
        g_angle_error_msg = msg
def distance_error_callback(msg):
    global g_distance_error_msg
    with g_lock:
        g_distance_error_msg = msg
def publish_text(event):
    with g_lock:
        if g_distance_error_msg and g_angle_error_msg:
            text_interface.publish("""RMS Distance Error: {0}
RMS Angular Error: {1}""".format(g_distance_error_msg.data, g_angle_error_msg.data))


if __name__ == "__main__":
    rospy.init_node("tracking_info")
    text_interface = OverlayTextInterface("~text")
    sub = rospy.Subscriber("~rms_angle_error", Float32, angle_error_callback)
    sub2 = rospy.Subscriber("~rms_distance_error", Float32, distance_error_callback)
    rospy.Timer(rospy.Duration(0.1), publish_text)
    rospy.spin()
