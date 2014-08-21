#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest("posedetection_msgs")
roslib.load_manifest("dynamic_tf_publisher")
from posedetection_msgs.msg import ObjectDetection
from posedetection_msgs.msg import Object6DPose
from dynamic_tf_publisher.srv import *
import tf
import numpy

class ObjectDetectionTfPublisher():
    def __init__(self):
        self.subscriber = rospy.Subscriber("ObjectDetection", ObjectDetection, self.callback);
        self.frame_id = rospy.get_param("~frame_id", "object")
        self.latest_publish = rospy.get_param("~latest_publish", False)
        if self.latest_publish:
            rospy.logerr("latest publish")
        self.init_object_messages()

    def init_object_messages(self):
        self.object_messages = None

        if rospy.has_param('~checker_board_params/position_x'):
            position_x = rospy.get_param('~checker_board_params/position_x', 0)
            position_y = rospy.get_param('~checker_board_params/position_y', 0)
            position_z = rospy.get_param('~checker_board_params/position_z', 0)
            orientation_x = rospy.get_param('~checker_board_params/orientation_x', 0)
            orientation_y = rospy.get_param('~checker_board_params/orientation_y', 0)
            orientation_z = rospy.get_param('~checker_board_params/orientation_z', 0)
            orientation_w = rospy.get_param('~checker_board_params/orientation_w', 0)
            rospy.logerr("objectdetection_tf_publisher load the calibration params :")
            rospy.logerr("    pos : %f %f %f orientation %f %f %f %f", position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w)

            loaded_object_messages = ObjectDetection()
            loaded_object = Object6DPose()
            loaded_object.pose.position.x = position_x
            loaded_object.pose.position.y = position_y
            loaded_object.pose.position.z = position_z
            loaded_object.pose.orientation.x = orientation_x
            loaded_object.pose.orientation.y = orientation_y
            loaded_object.pose.orientation.z = orientation_z
            loaded_object.pose.orientation.w = orientation_w

            loaded_object_messages.objects = [loaded_object]
            loaded_object_messages.header.frame_id = rospy.get_param('~checker_board_params/header_frame')
            self.object_messages = loaded_object_messages
        else:
            rospy.logerr("No parameters (~checker_board_params)  was found for object detection")

    def callback(self, msg):
        if  msg.objects or ( not self.latest_publish):
            self.object_messages = msg

    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.object_messages:
                for detected_object in self.object_messages.objects:
                    br = tf.TransformBroadcaster()
                    br.sendTransform((detected_object.pose.position.x,
                                     detected_object.pose.position.y,
                                     detected_object.pose.position.z),
                                     (detected_object.pose.orientation.x,
                                      detected_object.pose.orientation.y,
                                      detected_object.pose.orientation.z,
                                      detected_object.pose.orientation.w),
                                     rospy.get_rostime(),
                                     self.frame_id,
                                     self.object_messages.header.frame_id,
                                     )
            r.sleep()

if __name__== '__main__':
    rospy.init_node('objectdetection_tf_publisher', anonymous=True)
    object_detection_tf_publisher = ObjectDetectionTfPublisher()
    object_detection_tf_publisher.run()
