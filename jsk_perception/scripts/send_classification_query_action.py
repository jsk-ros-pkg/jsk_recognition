#!/usr/bin/env python

import actionlib
import jsk_recognition_msgs.msg
import rospy
from sensor_msgs.msg import Image

def cb(image):
    ac = actionlib.SimpleActionClient("/clip/clip_server" , jsk_recognition_msgs.msg.ClassificationTaskAction)
    ac.wait_for_server()
    goal = jsk_recognition_msgs.msg.ClassificationTaskGoal()
    goal.image = image
    goal.queries = ["human", "apple", "book"]
    ac.send_goal(goal)
    ac.wait_for_result()
    print(ac.get_result())

rospy.init_node("test_classification_action")
sub = rospy.Subscriber("/usb_cam/image_raw", Image, cb)
rospy.spin()
