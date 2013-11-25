#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest("posedetection_msgs")
roslib.load_manifest("dynamic_tf_publisher")
from posedetection_msgs.msg import ObjectDetection
from dynamic_tf_publisher.srv import *
import tf

def callback(msg, subscriber):
    for detected_object in msg.objects:
        br = tf.TransformBroadcaster()
        br.sendTransform((detected_object.position.x,
                          detected_object.position.y,
                          detected_object.position.z),
                         (detected_object.orientation.x,
                          detected_object.orientation.y,
                          detected_object.orientation.z,
                          detected_object.orientation.w),
                         rospy.Time.now(),
                         "object_" + msg.header.seq,
                         msg.header.frame_id)
#        subscriber.unregister()

if __name__== '__main__':
    rospy.init_node('objectdetection_tf_publisher', anonymous=True)
    subscriber = None
    subscriber = rospy.Subscriber("ObjectDetection", ObjectDetection, callback, subscriber)
    rospy.spin()
