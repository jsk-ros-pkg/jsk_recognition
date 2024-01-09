#!/usr/bin/env python

import actionlib
import rospy
from jsk_recognition.msg import VQATaskAction, VQATaskGoal

if __name__ == "__main__":
    rospy.init_node("vqa_interpreter")

    client = actionlib.SimpleActionClient("/vqa/inference_server", VQATaskAction)

    while not rospy.is_shutdown():
        question = input("Enter a question: ")
        if question == "exit":
            break
        goal = VQATaskGoal()
        goal.questions = [question]
        client.send_goal(goal)
        if client.wait_for_result(timeout=rospy.Duration(30.0)):
            result = client.get_result()
            print(result)
        else:
            print("Timeout")
