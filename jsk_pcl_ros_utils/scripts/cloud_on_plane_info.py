#!/usr/bin/env python
import rospy
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import Float32
from threading import Lock
from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface
from jsk_recognition_msgs.msg import BoolStamped

# workaround until https://github.com/jsk-ros-pkg/jsk_visualization/pull/864 is merged and released.
class OverlayTextInterface_fix(OverlayTextInterface):
    def publish(self, text):
        msg = OverlayText()
        msg.text = text
        msg.width = int(self.config.width)
        msg.height = int(self.config.height)
        msg.top = int(self.config.top)
        msg.left = int(self.config.left)
        msg.fg_color.a = self.config.fg_alpha
        msg.fg_color.r = self.config.fg_red
        msg.fg_color.g = self.config.fg_green
        msg.fg_color.b = self.config.fg_blue
        msg.bg_color.a = self.config.bg_alpha
        msg.bg_color.r = self.config.bg_red
        msg.bg_color.g = self.config.bg_green
        msg.bg_color.b = self.config.bg_blue
        msg.text_size = self.config.text_size
        self.pub.publish(msg)


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
    text_interface = OverlayTextInterface_fix("~text")
    sub = rospy.Subscriber("~input", BoolStamped, callback)
    rospy.Timer(rospy.Duration(0.1), publish_text)
    rospy.spin()
