#/usr/bin/env python
import cv2

import rospy
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3, PoseStamp


def cloud_cb():
    pub = rospy.Publisher("~output", Vector3, queue_size=1)
    pub.publish(image_message)

if __name__ == "__main__":
    rospy.init_node("fisheye_ray")
    file_name = rospy.get_param("~file_name", "image.png")
    rospy.Subscriber("clicked_point", PoseStamp, cloud_cb)
    rospy.spin()
