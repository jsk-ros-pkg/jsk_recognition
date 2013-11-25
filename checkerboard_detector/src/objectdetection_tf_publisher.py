#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest("posedetection_msgs")
roslib.load_manifest("dynamic_tf_publisher")
from posedetection_msgs.msg import ObjectDetection
from dynamic_tf_publisher.srv import *

def callback(msg):
    for detected_object in msg.objects:
        rospy.wait_for_service('set_dynamic_tf')
        try:
            execute_service = rospy.ServiceProxy('set_dynamic_tf', SetDynamicTF)
            srv = SetDynamicTFRequest()
            srv.freq = 100
            srv.cur_tf.transform.translation.x = detected_object.pose.position.x
            srv.cur_tf.transform.translation.y = detected_object.pose.position.y
            srv.cur_tf.transform.translation.z = detected_object.pose.position.z
            srv.cur_tf.transform.rotation.x = detected_object.pose.orientation.x
            srv.cur_tf.transform.rotation.y = detected_object.pose.orientation.y
            srv.cur_tf.transform.rotation.z = detected_object.pose.orientation.z
            srv.cur_tf.transform.rotation.w = detected_object.pose.orientation.w
            srv.cur_tf.header = msg.header

            response = execute_service(srv)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__== '__main__':
    rospy.init_node('objectdetection_tf_publisher', anonymous=True)
    rospy.Subscriber("ObjectDetection", ObjectDetection, callback)
    rospy.spin()
