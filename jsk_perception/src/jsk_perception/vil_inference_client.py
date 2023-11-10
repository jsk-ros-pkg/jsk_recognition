#!/usr/bin/env python
import abc
import base64
import json

import actionlib
import requests
import rospy
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from jsk_perception.cfg import ClassificationConfig, VQAConfig
from jsk_recognition_msgs.msg import (ClassificationResult,
                                      ClassificationTaskAction,
                                      ClassificationTaskFeedback,
                                      ClassificationTaskResult,
                                      QuestionAndAnswerText, VQAResult,
                                      VQATaskAction, VQATaskFeedback,
                                      VQATaskResult)
from requests.exceptions import ConnectionError
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

import cv2


class DockerInferenceClientBase(object):
    def __init__(self, action,
                 server_config,
                 result_topic,
                 action_feedback,
                 action_result,
                 app_name):
        # inference server configuration
        self.host = rospy.get_param("~host", default="localhost")
        self.port = rospy.get_param("~port", default=8888)
        self.app_name = app_name
        # cv bridge
        self._bridge = CvBridge()
        # default inference image
        self.default_img = None
        # ROS
        self.transport_hint = rospy.get_param('~image_transport', 'raw')
        if self.transport_hint == 'compressed':
            self.image_sub = rospy.Subscriber(
                "{}/compressed".format(rospy.resolve_name('~image')),
                CompressedImage,
                callback=self.topic_cb,
                queue_size=1,
                buff_size=2**26
            )

        else:
            self.image_sub = rospy.Subscriber("~image", Image,
                                              callback=self.topic_cb,
                                              queue_size=1,
                                              buff_size=2**26)
        self.result_topic_type = result_topic
        self.result_pub = rospy.Publisher("~result", result_topic, queue_size=1)
        if self.transport_hint == 'compressed':
            self.image_pub = rospy.Publisher("~result/image/compressed", CompressedImage, queue_size=1)
        else:
            self.image_pub = rospy.Publisher("~result/image", Image, queue_size=1)
        self.vis_pub = rospy.Publisher("~visualize", String, queue_size=1)
        self.action_server = actionlib.SimpleActionServer("~inference_server",
                                                          action,
                                                          execute_cb=self.action_cb,
                                                          auto_start=False)
        self.action_feedback = action_feedback
        self.action_result = action_result
        self.reconfigure_server = Server(server_config, self.config_cb)
        self.action_server.start()

    def ros_img_to_base(self, ros_img):
        if type(ros_img) is CompressedImage:
            cv_img = self._bridge.compressed_imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
        elif type(ros_img) is Image:
            cv_img = self._bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
        else:
            raise RuntimeError("Unknown type {}".format(type(ros_img)))
        # convert to base64
        encimg = cv2.imencode(".png", cv_img)[1]
        img_str = encimg.tostring()
        img_byte = base64.b64encode(img_str).decode("utf-8")
        return img_byte

    def config_cb(self, config, level):
        self.config = config
        return config

    @abc.abstractmethod
    def topic_cb(self, msg):
        pass

    def action_cb(self, goal):
        success = True
        result = self.action_result()
        feedback = self.action_feedback()
        if goal.image.data and (not goal.compressed_image.data):
            image = goal.image
            # result.result.image = image
        elif (not goal.image.data) and goal.compressed_image.data:
            image = goal.compressed_image
            # result.result.compressed_image = image
        elif goal.image.data and goal.image.compressed_image.data:
            rospy.logerr("Both image and compressed image can not be added simultaneously")
            return
        else:
            rospy.loginfo("No images in goal message, so using subscribed image topic instead")
            image = self.default_img
        queries = self.create_queries(goal)
        try:
            result.result = self.inference(image, queries)
        except Exception as e:
            rospy.logerr(str(e))
            feedback.status = str(e)
            success = False
        finally:
            self.action_server.publish_feedback(feedback)
            result.done = success
            self.action_server.set_succeeded(result)

    @abc.abstractmethod
    def create_queries(self, goal):
        pass

    @abc.abstractmethod
    def inference(self, img_msg, queries):
        pass

    def send_request(self, content):
        url = "http://{}:{}/{}".format(self.host, str(self.port), self.app_name)
        try:
            response = requests.post(url, data=content)
        except ConnectionError as e:
            rospy.logwarn_once("Cannot establish the connection with API server. Is it running?")
            raise e
        else:
            if response.status_code == 200:
                return response
            else:
                err_msg = "Invalid http status code: {}".format(str(response.status_code))
                rospy.logerr(err_msg)
                raise RuntimeError(err_msg)


class ClipClientNode(DockerInferenceClientBase):
    def __init__(self):
        DockerInferenceClientBase.__init__(self,
                                           ClassificationTaskAction,
                                           ClassificationConfig,
                                           ClassificationResult,
                                           ClassificationTaskFeedback,
                                           ClassificationTaskResult,
                                           "inference")

    def topic_cb(self, data):
        if not self.config: rospy.logwarn("No queries"); return
        if not self.config.queries: rospy.logwarn("No queries"); return
        queries = self.config.queries.split(";")
        try:
            msg = self.inference(data, queries)
        except Exception: return
        # publish debug image
        self.image_pub.publish(data)
        # publish classification result
        msg.header = data.header
        self.result_pub.publish(msg)
        # publish probabilities result as string
        vis_msg = ""
        for i, label in enumerate(msg.label_names):
            vis_msg += "{}: {:.2f}% ".format(label, msg.probabilities[i]*100)
        vis_msg += "\n\nCosine Similarity\n"
        for i, label in enumerate(msg.label_names):
            vis_msg += "{}: {:.4f} ".format(label, msg.label_proba[i])
        self.vis_pub.publish(vis_msg)

    def create_queries(self, goal):
        return goal.queries

    def inference(self, img_msg, queries):
        img_byte = self.ros_img_to_base(img_msg)
        req = json.dumps({"image": img_byte,
                          "queries": queries}).encode("utf-8")
        response = self.send_request(req)
        result_dic = json.loads(response.text)["results"]
        labels = []
        probabilities = []
        similarities = []
        for r in result_dic:
            labels.append(r["question"])
            probabilities.append(float(r["probability"]))
            similarities.append(float(r["similarity"]))
        labels = [label for _,label in sorted(zip(probabilities, labels), reverse=True)]
        probabilities.sort(reverse=True)
        similarities.sort(reverse=True)
        # build ClassificationResult message
        msg = self.result_topic_type()
        msg.labels = list(range(len(labels)))
        msg.label_names = labels
        msg.label_proba = similarities     # cosine similarities
        msg.probabilities = probabilities   # sum(probabilities) is 1
        msg.classifier = 'clip'
        msg.target_names = queries
        return msg


class OFAClientNode(DockerInferenceClientBase):
    def __init__(self):
        self.vqa_type = rospy.get_param("~vqa_type", default="caption") # caption, vqa_gen. caption is better than vqa_gen in OFA
        if self.vqa_type not in ["caption", "vqa_gen"]:
            raise RuntimeError("VQA type must be caption or vqa_gen")
        DockerInferenceClientBase.__init__(self,
                                           VQATaskAction,
                                           VQAConfig,
                                           VQAResult,
                                           VQATaskFeedback,
                                           VQATaskResult,
                                           self.vqa_type)

    def create_queries(self, goal):
        return goal.questions

    def inference(self, img_msg, queries):
        img_byte = self.ros_img_to_base(img_msg)
        req = json.dumps({"image": img_byte,
                          "queries": queries}).encode("utf-8")
        response = self.send_request(req)
        json_result = json.loads(response.text)
        msg = self.result_topic_type()
        for result in json_result["results"]:
            result_msg = QuestionAndAnswerText()
            result_msg.question = result["question"]
            result_msg.answer = result["answer"]
            msg.result.append(result_msg)
        return msg

    def topic_cb(self, data):
        if not self.config.questions: rospy.logwarn("No questions"); return
        queries = self.config.questions.split(";")
        try:
            msg = self.inference(data, queries)
        except Exception: return
        self.image_pub.publish(data)
        self.result_pub.publish(msg)
        vis = ""
        for qa in msg.result:
            vis += "Q:{}\n A:{}\n".format(qa.question,
                                          qa.answer)
        self.vis_pub.publish(vis)
