#!/usr/bin/env python
import abc
import base64
import json

import actionlib
import requests
import rospy
import matplotlib
import matplotlib.cm
import numpy as np
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from jsk_perception.cfg import ClassificationConfig, VQAConfig
from jsk_recognition_msgs.msg import (ClassificationResult,
                                      ClassificationTaskAction,
                                      ClassificationTaskFeedback,
                                      ClassificationTaskResult,
                                      DetectionResult,
                                      DetectionTaskAction,
                                      DetectionTaskFeedback,
                                      DetectionTaskResult,
                                      Rect, RectArray,
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

    def ros_img_to_cv(self, ros_img, encoding="bgr8"):
        # convert to cv2
        if type(ros_img) is CompressedImage:
            cv_img = self._bridge.compressed_imgmsg_to_cv2(ros_img, desired_encoding=encoding)
        elif type(ros_img) is Image:
            cv_img = self._bridge.imgmsg_to_cv2(ros_img, desired_encoding=encoding)
        else:
            raise RuntimeError("Unknown type {}".format(type(ros_img)))
        return cv_img

    def ros_img_to_base(self, ros_img):
        cv_img = self.ros_img_to_cv(ros_img)
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

class DINOClientNode(DockerInferenceClientBase):
    def __init__(self):
        DockerInferenceClientBase.__init__(self,
                                           DetectionTaskAction,
                                           ClassificationConfig,
                                           DetectionResult,
                                           DetectionTaskFeedback,
                                           DetectionTaskResult,
                                           "detection")
        self.pub_class = rospy.Publisher('~class', ClassificationResult, queue_size=1)
        self.pub_rects = rospy.Publisher('~rects', RectArray, queue_size=1)
        self.pub_image = rospy.Publisher('~output/image', Image, queue_size=1)

    def topic_cb(self, data):
        if not self.config: rospy.logwarn("No queries"); return
        if not self.config.queries: rospy.logwarn("No queries"); return
        queries = self.config.queries.split(";")
        try:
            msg = self.inference(data, queries)
        except Exception: return
        # publish debug image
        self.image_pub.publish(data)
        # publish detection result
        msg.header = data.header
        self.result_pub.publish(msg)
        # publish probabilities result as string
        vis_msg = ""
        for i, label in enumerate(msg.classification.label_names):
            vis_msg += "{}: {:.2f}% ".format(label, msg.classification.probabilities[i]*100)
        self.vis_pub.publish(vis_msg)

    def create_queries(self, goal):
        return goal.queries

    def inference(self, img_msg, queries):
        img_byte = self.ros_img_to_base(img_msg)
        req = json.dumps({"image": img_byte,
                          "queries": queries}).encode("utf-8")
        response = self.send_request(req)
        result_dic = json.loads(response.text)["results"]

        boxes = []
        scores = []
        labels = []
        for r in result_dic:
            boxes.append(r["box"])
            scores.append(r["logit"])
            labels.append(r["phrase"])
        classification_msg = ClassificationResult(header=img_msg.header)
        classification_msg.labels = list(range(len(labels)))
        classification_msg.label_names = labels
        classification_msg.label_proba = scores     # cosine similarities
        classification_msg.probabilities = scores   # sum(probabilities) is 1
        classification_msg.classifier = 'dino'
        classification_msg.target_names = queries
        self.pub_class.publish(classification_msg)

        rect_msg = RectArray(header=img_msg.header)
        vis_img = self.ros_img_to_cv(img_msg, encoding="rgb8")
        cmap = matplotlib.cm.get_cmap('hsv')
        n = max(len(boxes) - 1, 10)
        rects = []
        for i in range(len(boxes)):
            box = boxes[i]
            rgba = np.array(cmap(1. * i / n))
            color = rgba[:3] * 255
            label_text = '{}, {:.2f}'.format(labels[i], scores[i])
            x_min = max(int(box[0]), 0)
            y_min = max(int(box[1]), 0)
            x_max = min(int(box[2]), vis_img.shape[1])
            y_max = min(int(box[3]), vis_img.shape[0])
            cv2.rectangle(
                vis_img, (x_min, y_min), (x_max, y_max),
                color, thickness=3, lineType=cv2.LINE_AA)
            cv2.putText(
                vis_img, label_text, (x_min, max(y_min - 10, 0)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color,
                thickness=2, lineType=cv2.LINE_AA)
            rect = Rect(
                x=x_min, y=y_min,
                width=x_max - x_min, height=y_max - y_min)
            rect_msg.rects.append(rect)

        self.pub_rects.publish(rect_msg)
        vis_msg = self._bridge.cv2_to_imgmsg(vis_img, 'rgb8')
        vis_msg.header = img_msg.header
        self.pub_image.publish(vis_msg)

        msg = self.result_topic_type()
        msg.classification = classification_msg
        msg.rects = rect_msg
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
