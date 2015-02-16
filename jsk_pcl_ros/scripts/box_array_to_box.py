#!/usr/bin/env python
import rospy

PKG='jsk_pcl_ros'

import imp
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

def boxarray_cb(boxarray):
    if (len(boxarray.boxes)==0):
        rospy.loginfo("box size 0")
        return
    BPub.publish(boxarray.boxes[0])


if __name__ == "__main__":
    rospy.init_node('transform_box', anonymous=True)
    BPub = rospy.Publisher('bounding_box', BoundingBox)
    rospy.Subscriber("bounding_box_array", BoundingBoxArray, boxarray_cb)
    rospy.spin()
