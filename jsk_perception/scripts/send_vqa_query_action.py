#!/usr/bin/env python

import actionlib
import jsk_recognition_msgs.msg
import rospy
from sensor_msgs.msg import Image

def cb(image):
    ac = actionlib.SimpleActionClient("/vqa/inference_server" , jsk_recognition_msgs.msg.VQATaskAction)
    ac.wait_for_server()
    goal = jsk_recognition_msgs.msg.VQATaskGoal()
    goal.image = image
    goal.questions = ["what does this image decribe?"]
    ac.send_goal(goal)
    ac.wait_for_result()
    print(ac.get_result())

rospy.init_node("test_vqa_action")
sub = rospy.Subscriber("/usb_cam/image_raw", Image, cb)
rospy.spin()
