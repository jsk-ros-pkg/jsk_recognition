#!/usr/bin/env python
import actionlib
import base64
import cv2
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from jsk_perception.cfg import ClassificationConfig
from jsk_recognition_msgs.msg import ClassificationTaskAction
from jsk_recognition_msgs.msg import ClassificationTaskGoal
from jsk_recognition_msgs.msg import ClassificationTaskFeedback
from jsk_recognition_msgs.msg import ClassificationTaskResult
from jsk_recognition_msgs.msg import ClassificationResult
import json
import requests
import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

bridge = CvBridge()

def ros_img_to_base(ros_img):
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

class ClipClientNode:
    def __init__(self):
        # inference server configuration
        self.host = rospy.get_param("~host", default="localhost")
        self.port = rospy.get_param("~port", default=8888)
        # clip task
        self.clip_image_sub = rospy.Subscriber("~image", Image,
                                                  callback=self.clip_topic_cb, queue_size=1, buff_size=2**26)
        self.clip_result_pub = rospy.Publisher("~clip_result", ClassificationResult, queue_size=1)
        self.clip_image_pub = rospy.Publisher("~clip_result/image", Image, queue_size=1) # add asked image publisher for slow inference
        self.clip_vis_pub = rospy.Publisher("~clip_result/probabilities/visualize", String, queue_size=1)
        self.clip_as = actionlib.SimpleActionServer("~clip_server",
                                                    ClassificationTaskAction,
                                                    execute_cb=self.clip_action_cb,
                                                    auto_start=False)
        self.clip_as.start()
        self.default_img = None
        # dynamic reconfigure
        self._reconfigure_server = Server(ClassificationConfig, self.config_cb)

    def config_cb(self, config, level):
        self.query = config.queries
        return config

    def clip_topic_cb(self, data):
        img_byte = ros_img_to_base(data)
        queries = self.query.split(";")

        msg = self.get_classification_result(img_byte, queries)

        if msg:
            # publish debug image
            self.clip_image_pub.publish(data)

            # publish classification result
            msg.header = data.header
            self.clip_result_pub.publish(msg)

            # publish probabilities result as string
            vis_msg = ""
            for i, label in enumerate(msg.labels):
                vis_msg += "{}: {:.2f}% ".format(label, msg.probabilities[i]*100)
            self.clip_vis_pub.publish(vis_msg)
        else:
            rospy.logerr("Invalid http status code: {}".format(str(response.status_code)))

    def clip_action_cb(self, goal):
        success = True
        feedback = ClassificationTaskFeedback()
        result = ClassificationTaskResult()
        if goal.image.data and (not goal.compressed_image.data):
            image = goal.image
        elif (not goal.image.data) and goal.compressed_image.data:
            image = goal.compressed_image
        elif goal.image.data and goal.image.compressed_image.data:
            rospy.logerr("Both image and compressed image can not be added simultaneously")
            return
        else:
            rospy.loginfo("No images in goal message, so using subscribed image topic instead")
            image = self.default_img
        img_byte = ros_img_to_base(image)
        # creqte request
        req = json.dumps({"image": img_byte,
                          "queries": goal.queries}).encode("utf-8")
        try:
            msg = self.get_classification_result(img_byte, goal.queries)
            if msg:
                msg.header = goal.image.header
                result.result = msg
            else:
                rospy.logerr("Invalid http status code: {}".format(str(response.status_code)))
                return
        except Exception as e:
            rospy.logerr(str(e))
            feedback.status = str(e)
            success = False
        finally:
            self.clip_as.publish_feedback(feedback)
            result.done = success
            self.clip_as.set_succeeded(result)


    def get_classification_result(self, img_byte, queries):
        req = json.dumps({"image": img_byte,
                          "queries": queries}).encode("utf-8")
        response = self.send_request(req)
        if response.status_code == 200:
            result_dic = json.loads(response.text)["results"]
            labels = []
            probabilities = []
            for r in result_dic:
                labels.append(r["question"])
                probabilities.append(float(r["probability"]))
            labels = [label for _,label in sorted(zip(probabilities, labels), reverse=True)]
            probabilities.sort(reverse=True)
            # build ClassificationResult message
            msg = ClassificationResult()
            msg.labels = list(range(len(labels)))
            msg.label_names = labels
            msg.probabilities = probabilities
            msg.classifier = 'clip'
            msg.target_names = queries
            return msg
        else:
            rospy.logerr("Invalid http status code: {}".format(str(response.status_code)))
            return None

    def send_request(self, content):
        url = "http://{}:{}/inference".format(self.host, str(self.port))
        response = requests.post(url, data=content)
        return response

def main():
    rospy.init_node("clip")
    node = ClipClientNode()
    rospy.spin()

if __name__ == "__main__":
    main()
