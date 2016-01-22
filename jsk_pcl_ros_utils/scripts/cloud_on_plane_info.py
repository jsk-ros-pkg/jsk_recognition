#!/usr/bin/env python
import rospy
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import Float32
from threading import Lock
from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface
from jsk_recognition_msgs.msg import BoolStamped

g_lock = Lock()
g_msg = None
def callback(msg):
    global g_msg
    with g_lock:
        g_msg = msg
def publish_text(event):
    with g_lock:
        if g_msg:
            if g_msg.data:
                text_interface.publish('Is Object On Plane?: ON')
            else:
                text_interface.publish('Is Object On Plane?: <span style="color: red">Off</span>')


if __name__ == "__main__":
    rospy.init_node("cloud_on_plane_info")
    text_interface = OverlayTextInterface("~text")
    sub = rospy.Subscriber("~input", BoolStamped, callback)
    rospy.Timer(rospy.Duration(0.1), publish_text)
    rospy.spin()
