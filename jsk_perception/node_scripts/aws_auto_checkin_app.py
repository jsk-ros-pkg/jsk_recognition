#!/usr/bin/env python

##
## Modified from awslabs/auto-check-in-app
##
## https://github.com/awslabs/auto-check-in-app/blob/master/source/frontend/controller.py
##
##
## Author: Yuli-disney <yulikamiya@gmail.com>
##         Kei Okada <kei.okada@gmail.com>

from __future__ import division

import rospy

import message_filters
from sensor_msgs.msg import CompressedImage, Image
from opencv_apps.msg import FaceArrayStamped, Face, Rect
from jsk_recognition_msgs.msg import RectArray
from jsk_recognition_msgs.msg import ClassificationResult

import numpy as np

import boto3
import cv2
from cv_bridge import CvBridge
import datetime
import json
import requests
import math
import sys

from jsk_topic_tools import ConnectionBasedTransport


class AutoCheckIn(ConnectionBasedTransport):

    def __init__(self):
        super(AutoCheckIn, self).__init__()

        env_path = rospy.get_param('~env_path', 'env.json')
        rospy.loginfo("Loading AutoCheckin env variables from {}".format(env_path))
        try:
            with open(env_path) as env_json:
                env = json.load(env_json)
        except IOError:
            rospy.logerr('Cannot open "{}".\nCopy "default.env.json" file as a new file called "env.json" and edit parameters in it.'.format(env_path))
            sys.exit(1)

        try:
            self.FACE_AREA_THRESHOLD = env['FaceAreaThreshold']
            self.FACE_SIMILARITY_THRESHOLD = env['FaceSimilarityThreshold']
            self.COLLECTION_ID = env['CollectionId']
            self.DYNAMODB_TABLE = env['DynamodbTable']
            self.MAX_FACES = env['MaxFaces']
            region_name = env['Region']
        except KeyError:
            print('Invalid config file')
            raise

        aws_credentials_path = rospy.get_param('~aws_credentials_path', 'aws.json')
        rospy.loginfo("Loading AWS credentials from {}".format(aws_credentials_path))
        try:
            with open(aws_credentials_path) as aws_json:
                aws_credentials = json.load(aws_json)
        except IOError:
            rospy.logerr('Cannot open "{}".\n Please put region/aws_access_key_id/aws_secret_access_key to aws.json.'.format(aws_credentials_path))
            sys.exit(1)

        try:
            aws_access_key_id = aws_credentials['aws_access_key_id']
            aws_secret_access_key = aws_credentials['aws_secret_access_key']
        except KeyError:
            print('Invalid config file')
            raise

        self.rekognition = boto3.client(
            'rekognition',
            aws_access_key_id=aws_access_key_id,
            aws_secret_access_key=aws_secret_access_key,
            region_name=region_name)

        self.dynamodb =  boto3.resource(
            'dynamodb',
            aws_access_key_id=aws_access_key_id,
            aws_secret_access_key=aws_secret_access_key,
            region_name=region_name)
        self.dynamodb_table = self.dynamodb.Table(self.DYNAMODB_TABLE)

        self.always_publish = rospy.get_param('~always_publish', True)
        rospy.loginfo("Publish even if face is not found : {}".format(self.always_publish))

        self.use_window = rospy.get_param('~use_window', False)
        rospy.loginfo("Launch image window : {}".format(self.use_window))

        self.bridge = CvBridge()
        self.transport_hint = rospy.get_param('~image_transport', 'compressed')
        rospy.loginfo("Using transport {}".format(self.transport_hint))

        self.classifier_name = rospy.get_param(
            "~classifier_name", rospy.get_name())
        self.target_names = self.get_label_names()
        self.target_name_to_label_index = {}
        for i, name in enumerate(self.target_names):
            self.target_name_to_label_index[name] = i

        self.name_pub = self.advertise('face_name', FaceArrayStamped, queue_size=1)
        self.pub_rects = self.advertise("~output/rects", RectArray,
                                        queue_size=1)
        self.pub_class = self.advertise("~output/class", ClassificationResult,
                                        queue_size=1)
        if self.transport_hint == 'compressed':
            self.orig_image_pub = self.advertise('~image/compressed', CompressedImage, queue_size=1)
        else:
            self.orig_image_pub = self.advertise('~image', Image, queue_size=1)
        #
        # To process latest message, we need to set buff_size must be large enough.
        # we need to set buff_size larger than message size to use latest message for callback
        # 640*480(image size) / 5 (expected compressed rate) *
        #            70 (number of message need to be drop 70 x 30msec = 2100msec processing time)
        #
        # c.f. https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/
        #
        self.buff_size = rospy.get_param('~buff_size', 640 * 480 * 3 // 5 * 70)
        rospy.loginfo("rospy.Subscriber buffer size : {}".format(self.buff_size))

    def subscribe(self):
        if self.transport_hint == 'compressed':
            self.image_sub = message_filters.Subscriber('{}/compressed'.format(rospy.resolve_name('image')), CompressedImage, buff_size=self.buff_size)
        else:
            self.image_sub = message_filters.Subscriber('image', Image, buff_size=self.buff_size)
        self.roi_sub = message_filters.Subscriber('face_roi', FaceArrayStamped)
        self.subs = [self.image_sub, self.roi_sub]
        queue_size = rospy.get_param('~queue_size', 1)
        approximate_sync = rospy.get_param('~approximate_sync', True)
        if approximate_sync:
            slop = rospy.get_param('~slop', 1.0)
            self.ts = message_filters.ApproximateTimeSynchronizer(
                self.subs,
                queue_size, slop, allow_headerless=True)
        else:
            self.ts = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        self.ts.registerCallback(self.callback)
        rospy.loginfo("To process latest incomming message, use approximate_sync with queue_size == 1 is recommended")
        rospy.loginfo("  approximate_sync : {}".format(approximate_sync))
        rospy.loginfo("  queue_size : {}".format(queue_size))
        rospy.loginfo("Waiting for {} and {}".format(self.image_sub.name, self.roi_sub.name))

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def get_label_names(self):
        items = self.dynamodb_table.scan()['Items']
        return list(set([item["Name"] for item in items]))

    def findface(self, face_image):
        area = face_image.shape[0] * face_image.shape[1]
        if area < self.FACE_AREA_THRESHOLD / 10:
            return None
        if area > self.FACE_AREA_THRESHOLD * 2:
            # resize
            ratio = math.sqrt(area / (self.FACE_AREA_THRESHOLD * 2))
            face_image = cv2.resize(face_image, (int(
                face_image.shape[1] / ratio), int(face_image.shape[0] / ratio)))

        _, encoded_face_image = cv2.imencode('.jpg', face_image)

        # Call API
        try:
            res = self.rekognition.search_faces_by_image(
                CollectionId=self.COLLECTION_ID, Image={'Bytes': encoded_face_image.tobytes()},
                FaceMatchThreshold=self.FACE_SIMILARITY_THRESHOLD, MaxFaces=self.MAX_FACES)
            return res
        except self.rekognition.exceptions.InvalidParameterException as e:
            rospy.logdebug("No faces detected")
        except Exception as e:
            rospy.logerr(e)

        return None

    def callback(self, image, roi):
        start_time = rospy.Time.now()
        if self.transport_hint == 'compressed':
            # decode compressed image
            np_arr = np.fromstring(image.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if image.format.find("compressed rgb") > -1:
                img = img[:, :, ::-1]
        else:
            img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        if self.use_window:
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img_gray = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)

        faces = FaceArrayStamped()
        faces.header = image.header
        faces.faces = []
        for face in roi.faces:
            try:
                cx = int(face.face.x)
                cy = int(face.face.y)
                w =  int(face.face.width)
                h =  int(face.face.height)
            except Exception as e:
                rospy.logerr(e)
                return

            image_roi_slice = np.index_exp[cy - h // 2:cy + h // 2,
                                           cx - w // 2:cx + w // 2]
            ret = self.findface(img[image_roi_slice])
            if ret != None:
                if ret['FaceMatches'] != []:
                    try:
                      item = self.dynamodb_table.get_item(
                          Key={'RekognitionId':
                               ret['FaceMatches'][0]['Face']['FaceId']})
                      if not 'Item' in item:
                          rospy.loginfo("Item does not have FaceId {}".format(item))
                          continue
                      face_id = item['Item']['Name']
                      rospy.logdebug("FaceId: {}\n Similarity: {}".format(face_id, \
                                                                          ret['FaceMatches'][0]['Similarity']))
                      faces.faces.append(Face(face=Rect(cx - w // 2, cy - h // 2, w, h),
                                              label=face_id,
                                              confidence=ret['FaceMatches'][0]['Similarity'] / 100.0))
                    except KeyError as e:
                        rospy.logwarn(
                            "{}: Dynamodb does not have FaceID: {}".format(
                                e, ret['FaceMatches'][0]['Face']['FaceID']))

            if self.use_window: # copy colored face rectangle to img_gray
                img_gray[image_roi_slice] = img[image_roi_slice]

        # is always_publish is False, publish results only when the face is not detected
        if not self.always_publish and len(faces.faces) <= 0:
            return

        self.name_pub.publish(faces)

        cls_msg = ClassificationResult(
            header=image.header,
            classifier=self.classifier_name,
            target_names=self.target_names,
            labels=[self.target_name_to_label_index[face.label]
                    for face in faces.faces],
            label_names=[face.label for face in faces.faces],
            label_proba=[face.confidence for face in faces.faces],
        )

        rects_msg = RectArray(header=image.header)
        for face in faces.faces:
            rects_msg.rects.append(face.face)
        self.pub_rects.publish(rects_msg)
        self.pub_class.publish(cls_msg)

        if self.orig_image_pub.get_num_connections() > 0:
            self.orig_image_pub.publish(image)

        if self.use_window:
            cv2.imshow(image._connection_header['topic'], img_gray)
            cv2.waitKey(1)

        rospy.loginfo("processing time {} on message taken at {} sec ago".format(
            (rospy.Time.now() - start_time).to_sec(),
            (rospy.Time.now() - image.header.stamp).to_sec()))


if __name__ == '__main__':
    rospy.init_node('aws_auto_checkin_service')
    auto = AutoCheckIn()
    rospy.spin()

