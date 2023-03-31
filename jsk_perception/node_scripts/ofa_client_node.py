#!/usr/bin/env python
import actionlib
import base64
import cv2
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from jsk_perception.cfg import VQAConfig
from jsk_recognition_msgs.msg import QuestionAndAnswerText
from jsk_recognition_msgs.msg import VQATaskAction
from jsk_recognition_msgs.msg import VQATaskFeedback
from jsk_recognition_msgs.msg import VQATaskResult
from jsk_recognition_msgs.msg import VQAResult
import json
import requests
from requests.exceptions import ConnectionError
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

def ros_img_to_base(ros_img, bridge):
    if type(ros_img) is CompressedImage:
        cv_img = bridge.compressed_imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
    elif type(ros_img) is Image:
        cv_img = bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
    else:
        raise RuntimeError("Unknown type {}".format(type(ros_img)))
    # convert to base64
    encimg = cv2.imencode(".png", cv_img)[1]
    img_str = encimg.tostring()
    img_byte = base64.b64encode(img_str).decode("utf-8")
    return img_byte

class OFAClientNode(object):
    def __init__(self):
        self._bridge = CvBridge()
        # inference server configuration
        self.host = rospy.get_param("~host", default="localhost")
        self.port = rospy.get_param("~port", default=8888)
        self.vqa_type = rospy.get_param("~vqa_type", default="caption") # caption, vqa_gen. caption is better than vqa_gen in OFA
        if self.vqa_type not in ["caption", "vqa_gen"]:
            raise RuntimeError("VQA type must be caption or vqa_gen")
        # subscriber
        self.vqa_image_sub = rospy.Subscriber("~vqa_image", Image,
                                              callback=self.vqa_topic_cb, queue_size=1, buff_size=2**26)
        # publisher
        self.vqa_result_pub = rospy.Publisher("~vqa_result", VQAResult, queue_size=1)
        self.vqa_image_pub = rospy.Publisher("~vqa_result/image", Image, queue_size=1) # add asked image publisher for slow inference
        self.vqa_multi_vis_pub = rospy.Publisher("~vqa_result/visualize", String, queue_size=1)
        # action server
        self.vqa_as = actionlib.SimpleActionServer("~vqa_server",
                                                   VQATaskAction,
                                                   execute_cb=self.vqa_action_cb,
                                                   auto_start=False)
        self.default_vqa_img = None
        self.vqa_as.start()
        # dynamic reconfigure
        self._reconfigure_server = Server(VQAConfig, self.config_cb)

    def config_cb(self, config, level):
        self.questions = config.questions
        return config

    def vqa_topic_cb(self, data):
        self.default_vqa_img = data
        if not self.questions:
            return
        img_byte = ros_img_to_base(data, self._bridge)
        queries = self.questions.split(";")
        req = json.dumps({"image": img_byte,
                          "queries": queries}).encode("utf-8")
        try:
            response = self.send_request(self.vqa_type, req)
        except ConnectionError as e:
            rospy.logwarn_once("Cannot establish the connection with API server. Is it running?")
        else:
            if response.status_code == 200:
                json_result = json.loads(response.text)
                msg = VQAResult()
                msg.image = data
                multi_vis = ""
                for result in json_result["results"]:
                    result_msg = QuestionAndAnswerText()
                    result_msg.question = result["question"]
                    result_msg.answer = result["answer"]
                    msg.result.append(result_msg)
                    multi_vis += "Q:{}\n A:{}\n".format(result["question"],
                                                        result["answer"])
                self.vqa_image_pub.publish(data)
                self.vqa_result_pub.publish(msg)
                self.vqa_multi_vis_pub.publish(multi_vis)
            else:
                rospy.logerr("Invalid http status code: {}".format(str(response.status_code)))

    def vqa_action_cb(self, goal):
        success = True
        feedback = VQATaskFeedback()
        result = VQATaskResult()
        if goal.image.data and (not goal.compressed_image.data):
            image = goal.image
            result.result.image = image
        elif (not goal.image.data) and goal.compressed_image.data:
            image = goal.compressed_image
            result.result.compressed_image = image
        elif goal.image.data and goal.image.compressed_image.data:
            rospy.logerr("Both image and compressed image can not be added simultaneously")
            return
        else:
            rospy.loginfo("No images in goal message, so using subscribed image topic instead")
            image = self.default_vqa_img
            result.result.image = image
        img_byte = ros_img_to_base(image, self._bridge)
        # create request
        queries = goal.questions if goal.questions else self.questions.split(";")
        req = json.dumps({"image": img_byte,
                          "queries": queries}).encode("utf-8")
        try:
            response = self.send_request(self.vqa_type, req) # FIXME or vqa_gen
            if response.status_code == 200:
                json_result = json.loads(response.text)["results"]
                for res in json_result:
                    msg = QuestionAndAnswerText()
                    msg.question = res["question"]
                    msg.answer = res["answer"]
                    result.result.result.append(msg)
            else:
                rospy.logerr("Invalid http status code: {}".format(str(response.status_code)))
                return
        except Exception as e:
            rospy.logerr(str(e))
            feedback.status = str(e)
            success = False
        finally:
            self.vqa_as.publish_feedback(feedback)
            result.done = success
            self.vqa_as.set_succeeded(result)

    def send_request(self, app_name, content):
        url = "http://{}:{}/{}".format(self.host, str(self.port), app_name)
        response = requests.post(url, data=content)
        return response

def main():
    rospy.init_node("ofa_ros_client")
    node = OFAClientNode()
    rospy.spin()

if __name__ == "__main__":
    main()
