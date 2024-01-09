#!/usr/bin/env python

import base64
from typing import Dict, Optional

import actionlib
import cv2
import numpy as np
import requests
import ros_numpy
import rospy
from dynamic_reconfigure.server import Server
from jsk_recognition_msgs.msg import (
    QuestionAndAnswerText,
    VQATaskAction,
    VQATaskGoal,
    VQATaskResult,
)
from sensor_msgs.msg import Image

from gpt4v_vqa.cfg import GPT4VConfig


class GPT4VClientNode(object):
    def __init__(self, api_key: str):
        self.api_key = api_key
        # Configuration
        self.max_height: int = 480
        self.max_width: int = 640
        self.detail_level: str = "low"
        # Node variables
        self.default_img: Optional[np.ndarray] = None
        self.sub = rospy.Subscriber("~image", Image, self._image_cb)
        self.param_srv = Server(GPT4VConfig, self.config_cb)
        self.ac = actionlib.SimpleActionServer(
            "~inference_server", VQATaskAction, self._ac_cb, auto_start=False
        )
        self.ac.start()

    def config_cb(self, config, level):
        """Dynamic reconfigure callback"""
        self.set_max_size(config["max_height"], config["max_width"])
        self.detail_level = config["detail_level"]
        return config

    def set_max_size(self, max_height: int, max_width: int):
        """Set max size of image to send to API

        Args:
            max_height (int): max height
            max_width (int): max width
        """
        self.max_height = max_height
        self.max_width = max_width

    def resize_image(self, image: np.ndarray) -> np.ndarray:
        """Resize image to maximum size configuration

        Args:
            image (np.ndarray): image to resize

        Returns:
            np.ndarray: resized image
        """
        height, width, num_channel = image.shape
        if height > self.max_height or width > self.max_width:
            scale = min(self.max_height / height, self.max_width / width)
            image = cv2.resize(
                image,
                (int(width * scale), int(height * scale)),
                interpolation=cv2.INTER_AREA,
            )
        return image

    def _image_cb(self, msg: Image):
        image = ros_numpy.numpify(msg)
        self.default_img = image

    def _ac_cb(self, goal: VQATaskGoal):
        """Action callback

        Args:
            goal (VQATaskAction): action goal
        """
        rospy.loginfo("Received goal")
        result = VQATaskResult()

        if len(goal.image.data) > 0:
            image = ros_numpy.numpify(goal.image)
        elif len(goal.compressed_image.data) > 0:
            rospy.logerr(f"Compressed image is not supported.")
            self.ac.set_aborted(result)
            return
        else:
            if self.default_img is not None:
                image = self.default_img
            else:
                rospy.logerr("Image is empty")
                self.ac.set_aborted(result)
                return
        image = self.resize_image(image)
        for question in goal.questions:
            response = self._get_multimodal_response(question, image)
            if response is None:
                rospy.logerr(f"Failed to get response from question {question}")
                continue
            if "choices" not in response or len(response["choices"]) == 0:
                rospy.logerr(f"No choices in response: {response}")
                continue
            answer = response["choices"][0]["message"]["content"]
            result.result.result.append(
                QuestionAndAnswerText(question=question, answer=answer)
            )
        if len(result.result.result) == 0:
            rospy.logerr("No answers found")
            self.ac.set_aborted(result)
            return
        else:
            self.ac.set_succeeded(result)

    def _get_multimodal_response(
        self,
        question: str,
        image: np.ndarray,
        max_tokens: int = 300,
        detail: str = "low",
    ) -> Optional[Dict]:
        """Get response from GPT-4-Vision API

        Args:
            question (str): question to ask
            image (np.ndarray): image to ask question about
            max_tokens (int, optional): max tokens to use for output. Defaults to 300. Which is about $0.01 at 2024-01-09. (See https://openai.com/pricing)
            detail (str, optional): detail level. Defaults to "low". See https://platform.openai.com/docs/guides/vision/managing-images for details.

        Returns:
            Dict: response from API"""
        base64_image = base64.b64encode(cv2.imencode(".jpg", image)[1]).decode("utf-8")
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}",
        }

        payload = {
            "model": "gpt-4-vision-preview",
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": question},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{base64_image}",
                                "defail": detail,
                            },
                        },
                    ],
                }
            ],
            "max_tokens": max_tokens,
        }
        try:
            response = requests.post(
                "https://api.openai.com/v1/chat/completions",
                headers=headers,
                json=payload,
            )
            return response.json()
        except requests.exceptions.RequestException as e:
            rospy.logerr(e)
            return None


if __name__ == "__main__":
    rospy.init_node("vqa")
    api_key = rospy.get_param("~api_key")
    GPT4VClientNode(api_key)
    rospy.spin()
