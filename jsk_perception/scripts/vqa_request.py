#!/usr/bin/env python

import argparse
import actionlib
import cv2
from cv_bridge import CvBridge
from jsk_recognition_msgs.msg import VQATaskAction, VQATaskActionGoal
import json
import rospy # for logging
import os

class VQAClient(object):
    def __init__(self):
        self.ac_caption_client = actionlib.SimpleActionClient("/vqa/vqa_server", VQATaskAction)
        self._bridge = CvBridge()

    def request(self, questions_path, image_path, output_dir=None):
        self.ac_caption_client.wait_for_server()
        caption_goal = VQATaskActionGoal()
        cv_image = cv2.imread(image_path)
        image = self._bridge.cv2_to_imgmsg(cv_image, "bgr8")
        caption_goal.goal.image = image
        with open(questions_path) as f:
            for q in f:
                caption_goal.goal.questions.append(q)
        self.ac_caption_client.send_goal(caption_goal.goal)
        self.ac_caption_client.wait_for_result()
        caption_result = self.ac_caption_client.get_result()
        result = {}
        for r in caption_result.result.result:
            result[r.question] = r.answer
        image_name = os.path.splitext(os.path.basename(image_path))[0]
        json_name = image_name + ".json"
        if output_dir:
            save_path = os.path.join(output_dir, json_name)
        else:
            save_path = os.path.join(os.path.dirname(image_path), json_name)
        with open(save_path, "w") as f:
            json.dump(result, f, indent=4, separators=(',', ': '))

parser = argparse.ArgumentParser(description="CLI interface for VQA action client")
parser.add_argument("questions_path", help="Question text file path of VQA input", type=str)
parser.add_argument("image_path", help="Image file path of VQA input", type=str)
parser.add_argument("-o", "--output", default=None)

args = parser.parse_args()

if __name__ == "__main__":
    rospy.init_node("vqa_request_client", anonymous=True)
    client = VQAClient()
    questions_path = args.questions_path
    image_path = args.image_path
    output = args.output
    client.request(questions_path, image_path, output)
