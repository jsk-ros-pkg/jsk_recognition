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
from sensor_msgs.msg import Image
from std_msgs.msg import String

def ros_img_to_base(ros_img, bridge):
    cv_img = bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
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
        # Caption
        self.caption_image_sub = rospy.Subscriber("~caption_image", Image,
                                                  callback=self.caption_topic_cb, queue_size=1, buff_size=2**26)
        self.caption_result_pub = rospy.Publisher("~caption_result", VQAResult, queue_size=1)
        self.caption_image_pub = rospy.Publisher("~caption_result/image", Image, queue_size=1) # add asked image publisher for slow inference
        self.caption_multi_vis_pub = rospy.Publisher("~caption_result/visualize", String, queue_size=1)
        self.caption_as = actionlib.SimpleActionServer("~caption_server",
                                                       VQATaskAction,
                                                       execute_cb=self.caption_action_cb,
                                                       auto_start=False)
        self.caption_as.start()
        self.default_caption_img = None
        # VQA
        self.vqa_image_sub = rospy.Subscriber("~vqa_image", Image,
                                              callback=self.vqa_topic_cb, queue_size=1, buff_size=2**26)
        self.vqa_result_pub = rospy.Publisher("~vqa_result", VQAResult, queue_size=1)
        self.vqa_image_pub = rospy.Publisher("~vqa_result/image", Image, queue_size=1) # add asked image publisher for slow inference
        self.vqa_multi_vis_pub = rospy.Publisher("~vqa_result/visualize", String, queue_size=1)
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

    def caption_topic_cb(self, data):
        self.default_caption_img = data
        if not self.questions:
            return
        self.default_caption_img = data
        img_byte = ros_img_to_base(data, self._bridge)
        queries = self.questions.split(";")
        req = json.dumps({"image": img_byte,
                          "queries": queries}).encode("utf-8")
        try:
            response = self.send_request("caption", req)
        except ConnectionError:
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
                self.caption_image_pub.publish(data)
                self.caption_result_pub.publish(msg)
                self.caption_multi_vis_pub.publish(multi_vis)
            else:
                rospy.logerr("Invalid http status code: {}".format(str(response.status_code)))

    def caption_action_cb(self, goal):
        success = True
        feedback = VQATaskFeedback()
        result = VQATaskResult()
        success = True
        if goal.image.data:
            result.result.image = goal.image
        else:
            rospy.loginfo("No images in goal message, so using subscribed image topic instead")
            result.result.image = self.default_caption_img
        img_byte = ros_img_to_base(result.result.image, self._bridge)
        # creqte request
        req = json.dumps({"image": img_byte,
                          "queries": goal.questions}).encode("utf-8")
        try:
            response = self.send_request("caption", req)
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
            self.caption_as.publish_feedback(feedback)
            result.done = success
            self.caption_as.set_succeeded(result)

    def vqa_topic_cb(self, data):
        self.default_vqa_img = data
        if not self.questions:
            return
        img_byte = ros_img_to_base(data, self._bridge)
        queries = self.questions.split(";")
        req = json.dumps({"image": img_byte,
                          "queries": queries}).encode("utf-8")
        try:
            response = self.send_request("vqa_gen", req)
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
        if goal.image.data:
            result.result.image = goal.image
        else:
            rospy.loginfo("No images in goal message, so using subscribed image topic instead")
            result.result.image = self.default_vqa_img
        img_byte = ros_img_to_base(result.result.image, self._bridge)
        # creqte request
        req = json.dumps({"image": img_byte,
                          "queries": goal.queries}).encode("utf-8")
        try:
            response = self.send_request("vqa_gen", req)
            if response.status_code == 200:
                json_result = json.loads(response.text)["results"]
                for res in json_result:
                    msg = QuestionAndAnswerText()
                    msg.question = res["question"]
                    msg.answer = res["answer"]
                    result.result.append(msg)
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
