#!/usr/bin/env python

# it depends on jsk_rviz_plugins

import rospy
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import Float32
from threading import Lock
from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface
from jsk_recognition_msgs.msg import TrackerStatus
g_lock = Lock()
g_tracker_status_msg = None
def tracker_status_callback(msg):
    global g_tracker_status_msg
    with g_lock:
        g_tracker_status_msg = msg
def publish_text(event):
    with g_lock:
        if g_tracker_status_msg:
            if g_tracker_status_msg.is_tracking:
                status_interface.publish('Tracker Status: <span style="color: red;">Tracking</span>')
            else:
                status_interface.publish('Tracker Status: Stable')


if __name__ == "__main__":
    rospy.init_node("tracker_status_info")
    status_interface = OverlayTextInterface("~text")
    sub = rospy.Subscriber("~input", TrackerStatus, tracker_status_callback)
    rospy.Timer(rospy.Duration(0.1), publish_text)
    rospy.spin()
