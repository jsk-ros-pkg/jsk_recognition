#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest("posedetection_msgs")
roslib.load_manifest("dynamic_tf_publisher")
from posedetection_msgs.msg import ObjectDetection
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PoseStamped
from dynamic_tf_publisher.srv import *
import tf

class ObjectDetectionTfPublisher():
    def __init__(self):

        if rospy.get_param('~use_simple_tf', False):
            self.br = tf.TransformBroadcaster()
            self.subscriber = rospy.Subscriber("ObjectDetection", ObjectDetection, self.simple_callback);
        else:
            self.init_object_messages()
            self.frame_id = rospy.get_param("~frame_id", "object")
            self.subscriber = rospy.Subscriber("ObjectDetection", ObjectDetection, self.callback);

    def init_object_messages(self):
        if rospy.has_param('~checker_board_params/position_x'):
            position_x = rospy.get_param('~checker_board_params/position_x', 0)
            position_y = rospy.get_param('~checker_board_params/position_y', 0)
            position_z = rospy.get_param('~checker_board_params/position_z', 0)
            orientation_x = rospy.get_param('~checker_board_params/orientation_x', 0)
            orientation_y = rospy.get_param('~checker_board_params/orientation_y', 0)
            orientation_z = rospy.get_param('~checker_board_params/orientation_z', 0)
            orientation_w = rospy.get_param('~checker_board_params/orientation_w', 0)
            rospy.loginfo("objectdetection_tf_publisher load the calibration params :")
            rospy.loginfo("    pos : %f %f %f orientation %f %f %f %f", position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w)

            self.send_dynamic_tf_request(position_x, position_y, position_z,
                                         orientation_x,orientation_y,orientation_z,orientation_w,
                                         rospy.get_param('~checker_board_params/header_frame'),
                                         self.frame_id
                                         )
        else:
            rospy.loginfo("No parameters (~checker_board_params)  was found for object detection")

    def simple_callback(self, msg):
        if  msg.objects:
            for detected_object in msg.objects:
                self.br.sendTransform((detected_object.pose.position.x, detected_object.pose.position.y, detected_object.pose.position.z),
                                      (detected_object.pose.orientation.x, detected_object.pose.orientation.y, detected_object.pose.orientation.z, detected_object.pose.orientation.w),
                                      msg.header.stamp,
                                      detected_object.type,
                                      msg.header.frame_id)

    def callback(self, msg):
        if  msg.objects:
            for detected_object in msg.objects:
                self.send_dynamic_tf_request(
                    detected_object.pose.position.x,
                    detected_object.pose.position.y,
                    detected_object.pose.position.z,
                    detected_object.pose.orientation.x,
                    detected_object.pose.orientation.y,
                    detected_object.pose.orientation.z,
                    detected_object.pose.orientation.w,
                    msg.header.frame_id,
                    self.frame_id
                    )

    def send_dynamic_tf_request(self, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w, parent_frame_id, child_frame_id):
        transform = Transform()
        transform.translation.x = pos_x
        transform.translation.y = pos_y
        transform.translation.z = pos_z
        transform.rotation.x = ori_x
        transform.rotation.y = ori_y
        transform.rotation.z = ori_z
        transform.rotation.w = ori_w

        set_tf_request = SetDynamicTFRequest()
        set_tf_request.freq = 10
        set_tf_request.cur_tf.header.stamp = rospy.get_rostime()
        set_tf_request.cur_tf.header.frame_id = parent_frame_id
        set_tf_request.cur_tf.child_frame_id = child_frame_id
        set_tf_request.cur_tf.transform = transform

        rospy.wait_for_service('/set_dynamic_tf')
        set_dynamic_tf = rospy.ServiceProxy('/set_dynamic_tf', SetDynamicTF)
        try:
            res = set_dynamic_tf(set_tf_request)
        except rospy.ServiceException as exc:
            print(("Service did not process request: " + str(exc)))

if __name__== '__main__':
    rospy.init_node('objectdetection_tf_publisher', anonymous=True)
    object_detection_tf_publisher = ObjectDetectionTfPublisher()
    rospy.spin()
