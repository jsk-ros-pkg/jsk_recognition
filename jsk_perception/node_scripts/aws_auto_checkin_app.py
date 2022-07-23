#!/usr/bin/env python

##
## Modified from awslabs/auto-check-in-app
##
## https://github.com/awslabs/auto-check-in-app/blob/master/source/frontend/controller.py
##
##
## Author: Yuli-disney <yulikamiya@gmail.com>
##         Kei Okada <kei.okada@gmail.com>

import rospy

import message_filters
from sensor_msgs.msg import CompressedImage
from opencv_apps.msg import FaceArrayStamped, Face, Rect

import numpy as np

import boto3
import cv2
import datetime
import json
import requests
import math

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
            self.COLLECTION_ID = "auto-check-in-app-face-collection"
            self.MAX_FACES = 1
            region_name = env['Region']
        except KeyError:
            print('Invalid config file')
            raise

        aws_credentials_path = rospy.get_param('~aws_credentials_path', '/tmp/aws.json')
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

        self.use_window = rospy.get_param('~use_window', False)
        rospy.loginfo("Launch image window : {}".format(self.use_window))

        self.name_pub = self.advertise('face_name', FaceArrayStamped, queue_size=1)

    def subscribe(self):
        self.image_sub = message_filters.Subscriber('{}/compressed'.format(rospy.resolve_name('image')), CompressedImage)
        self.roi_sub = message_filters.Subscriber('face_roi', FaceArrayStamped)
        self.subs = [self.image_sub, self.roi_sub]
        queue_size = rospy.get_param('~queue_size', 100)
        if rospy.get_param('~approximate_sync', True):
            slop = rospy.get_param('~slop', 1.0)
            self.ts = message_filters.ApproximateTimeSynchronizer(
                self.subs,
                queue_size, slop, allow_headerless=True)
        else:
            self.ts = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
            self.ts.registerCallback(self.callback)
        rospy.loginfo("Waiting for {} and {}".format(self.image_sub.name, self.roi_sub.name))

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

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
        except Exception as e:
            print(e)

        return None

    def callback(self, image, roi):
        # decode compressed image
        np_arr = np.fromstring(image.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image.format.find("compressed rgb") > -1:
            img = img[:, :, ::-1]

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

            ret = self.findface(img[cy-h/2:cy+h/2,cx-w/2:cx+w/2])
            if ret != None:
                if ret['FaceMatches'] != []:
                    rospy.loginfo(ret)
                    rospy.loginfo("FaceId: {}\n Similarity: {}".format(ret['FaceMatches'][0]['Face']['FaceId'], \
                                                                       ret['FaceMatches'][0]['Similarity']))
                    faces.faces.append(Face(face=Rect(cx, cy, w, h),
                                            label=ret['FaceMatches'][0]['Face']['FaceId'],
                                            confidence=ret['FaceMatches'][0]['Similarity']))

            if self.use_window: # copy colored face rectangle to img_gray
                img_gray[cy-h/2:cy+h/2,cx-h/2:cx+w/2] = img[cy-h/2:cy+h/2,cx-w/2:cx+w/2]

        self.name_pub.publish(faces)

        if self.use_window:
            cv2.imshow(image._connection_header['topic'], img_gray)
            cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('aws_auto_checkin_service')
    auto = AutoCheckIn()
    rospy.spin()

