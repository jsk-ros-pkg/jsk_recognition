#!/usr/bin/env python

import rospy

from gpt4v_vqa import VQAClient

if __name__ == "__main__":
    rospy.init_node("vqa_interpreter")

    client = VQAClient()
    client.wait_for_server()

    while not rospy.is_shutdown():
        question = input("Enter a question: ")
        if question == "exit":
            break
        result = client.vqa(question)
        print(result)
