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

class AutoCheckIn(object):

    def _get_id_token_by_cognito(self, username, password):
        client = boto3.client('cognito-idp', self.REGION)
        rospy.loginfo("Initiate User Auth for {}".format(username))
        response = client.initiate_auth(
            ClientId=self.COGNITO_USERPOOL_CLIENT_ID,
            AuthFlow='USER_PASSWORD_AUTH',
            AuthParameters={
                'USERNAME': username,
                'PASSWORD': password
            }
        )
        return response['AuthenticationResult']['IdToken']

    def __init__(self):
        rospy.init_node('aws_auto_checkin_service')
        rospy.loginfo("ROS node initialized as {}".format(rospy.get_name()))

        env_path = rospy.get_param('~env_path', 'env.json')
        rospy.loginfo("Loading AutoCheckin env variables from {}".format(env_path))
        try:
            with open(env_path) as env_json:
                env = json.load(env_json)
        except IOError:
            print('Cannot open "{}".\nCopy "default.env.json" file as a new file called "env.json" and edit parameters in it.'.format(env_path))

        try:
            self.API_ENDPOINT = env['ApiEndpoint']
            self.FACE_AREA_THRESHOLD = env['FaceAreaThreshold']
            self.NAME_TTL_SEC = env['NameTtlSec']
            self.FACE_SIMILARITY_THRESHOLD = env['FaceSimilarityThreshold']
            self.COGNITO_USERPOOL_ID = env['CognitoUserPoolId']
            self.COGNITO_USERPOOL_CLIENT_ID = env['CognitoUserPoolClientId']
            self.REGION = env['Region']
        except KeyError:
            print('Invalid config file')
            raise

        self.id_token = self._get_id_token_by_cognito(env['UserName'], env['UserPassword'])

        self.use_window = rospy.get_param('~use_window', False)
        rospy.loginfo("Launch image window : {}".format(self.use_window))

        self.name_pub = rospy.Publisher('face_name', FaceArrayStamped, queue_size=1)
        self.image_sub = message_filters.Subscriber('{}/compressed'.format(rospy.resolve_name('image')), CompressedImage)
        # we wan to use RegionOfInterest, but it message_filters requires
        # header information, so use CameraInfo
        self.roi_sub = message_filters.Subscriber('face_roi', FaceArrayStamped)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.roi_sub], 10, 1, allow_headerless = True)
        self.ts.registerCallback(self.callback)
        rospy.loginfo("Waiting for {} and {}".format(self.image_sub.name, self.roi_sub.name))

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
            endpoint = 'https://' + self.API_ENDPOINT
            t = datetime.datetime.utcnow()
            amz_date = t.strftime('%Y%m%dT%H%M%SZ')
            headers = {
                'Content-Type': 'image/jpg',
                'X-Amz-Date':amz_date,
                'Authorization': self.id_token
            }
            request_parameters = encoded_face_image.tostring()
            res = requests.post(endpoint, data=request_parameters, headers=headers).json()
            rospy.loginfo("responce : {}".format(res))
            # renponse samples:
            #      {'result': 'OK', 'name': 'hoge', 'similarity': 95.15}
            #      {'result': 'NO_MATCH', 'name': '', 'similarity': 0}
            #      {'result': 'INVALID', 'name': '', 'similarity': 0}

            result = res['result']
        except Exception as e:
            print(e)

        else:
            if result == 'OK':
                name = res['name']
                similarity = res['similarity']
                if similarity > self.FACE_SIMILARITY_THRESHOLD:
                    return res

        return None

    def callback(self, image, roi):
        # decode compressed image
        np_arr = np.fromstring(image.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image.format != "rgb8; jpeg compressed bgr8":
            img = img[:, :, ::-1]

        if self.use_window:
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img_gray = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)

        faces = FaceArrayStamped()
        faces.header = image.header
        faces.faces = []
        for face in roi.faces:
            cx = int(face.face.x)
            cy = int(face.face.y)
            w =  int(face.face.width)
            h =  int(face.face.height)

            ret = self.findface(img[cy-h/2:cy+h/2,cx-w/2:cx+w/2])
            if ret != None:
                faces.faces.append(Face(face=Rect(cx, cy, w, h),
                                        label=ret['name'],
                                        confidence=ret['similarity']))

            if self.use_window: # copy colored face rectangle to img_gray
                img_gray[cy-h/2:cy+h/2,cx-h/2:cx+w/2] = img[cy-h/2:cy+h/2,cx-w/2:cx+w/2]

        self.name_pub.publish(faces)

        if self.use_window:
            cv2.imshow(image._connection_header['topic'], img_gray)
            cv2.waitKey(1)


if __name__ == '__main__':
    auto = AutoCheckIn()
    rospy.spin()

