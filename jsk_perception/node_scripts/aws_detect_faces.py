#!/usr/bin/env python

#
# ROS driver for AWS Rekognition (detect_faces)
#
# https://github.com/awsdocs/amazon-rekognition-developer-guide/blob/master/doc_source/faces-detect-images.md
#
#
# Author: Kei Okada <k-okada@jsk.t.u-tokyo.ac.jp>
#

from __future__ import division

import rospy
from dynamic_reconfigure.server import Server
from jsk_perception.cfg import AWSDetectFacesConfig

from jsk_topic_tools import ConnectionBasedTransport
from geometry_msgs.msg import PoseArray, Pose, Point
from sensor_msgs.msg import CompressedImage, Image
from opencv_apps.msg import FaceArrayStamped, Face, Rect
from jsk_recognition_msgs.msg import ClassificationResult, PeoplePoseArray, PeoplePose
from tf.transformations import quaternion_from_euler

import numpy as np
import math

import boto3

import cv2
import cv_bridge

import json

import sys

COLORS = [
    (100, 100, 100),
    (100, 0, 0),
    (150, 0, 0),
    (200, 0, 0),
    (255, 0, 0),
    (100, 100, 0),
    (150, 150, 0),
    (200, 200, 0),
    (255, 255, 0),
    (0, 100, 50),
    (0, 150, 75),
    (0, 200, 100),
    (0, 255, 125),
    (0, 50, 100),
    (0, 75, 150),
    (0, 100, 200),
    (0, 125, 255),
    (100, 0, 100),
    (150, 0, 150),
    (200, 0, 200),
    (255, 0, 255),
]


class DetectFaces(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()

        aws_credentials_path = rospy.get_param('~aws_credentials_path', 'aws.json')
        rospy.loginfo("Loading AWS credentials from {}".format(aws_credentials_path))
        try:
            with open(aws_credentials_path) as f:
                aws_credentials = json.load(f)
        except IOError:
            rospy.logerr('Cannot open "{}".\n Please put region/aws_access_key_id/aws_secret_access_key to aws.json.'.format(aws_credentials_path))
            sys.exit(1)

        try:
            aws_access_key_id = aws_credentials['aws_access_key_id']
            aws_secret_access_key = aws_credentials['aws_secret_access_key']
            region_name = aws_credentials['region']
        except KeyError:
            print('Invalid config file')
            raise

        self.rekognition = boto3.client(
            'rekognition',
            aws_access_key_id=aws_access_key_id,
            aws_secret_access_key=aws_secret_access_key,
            region_name=region_name)

        self.bridge = cv_bridge.CvBridge()

        self.always_publish = rospy.get_param('~always_publish', True)
        rospy.loginfo("Publish even if face is not found : {}".format(self.always_publish))

        self.use_window = rospy.get_param('~use_window', False)
        rospy.loginfo("Launch image window : {}".format(self.use_window))

        # enable to change atributes from Dynamic Reconfigure
        self.attributes = rospy.get_param('~attributes', 'ALL')
        rospy.loginfo("Facial attributes to be returned : {}".format(self.attributes))
        self.old_config = {self.attributes: False}
        self.dynamic_reconfigure_server = Server(AWSDetectFacesConfig, self.reconfigure_callback)

        self.faces_pub = self.advertise('~faces', FaceArrayStamped, queue_size=1)
        self.poses_pub = self.advertise('~poses', PoseArray, queue_size=1)
        self.attributes_pub = self.advertise('~attributes', ClassificationResult, queue_size=1)
        self.landmarks_pub = self.advertise('~landmarks', PeoplePoseArray, queue_size=1)
        self.image_pub = self.advertise('~output', Image, queue_size=1)
        self.image_comp_pub = self.advertise('~output/compressed', CompressedImage, queue_size=1)
        self.orig_image_pub = self.advertise('~image/compressed', CompressedImage, queue_size=1)
        #
        # To process latest message, we need to set buff_size must be large enough.
        # we need to set buff_size larger than message size to use latest message for callback
        # 640*480(image size) / 5 (expected compressed rate) *
        #            10 (number of message need to be drop 10 x 30msec = 300msec processing time)
        #
        # c.f. https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/
        #
        self.buff_size = rospy.get_param('~buff_size', 640 * 480 * 3 // 5 * 10)
        rospy.loginfo("rospy.Subscriber buffer size : {}".format(self.buff_size))

    def subscribe(self):
        self.image_sub = rospy.Subscriber('{}/compressed'.format(rospy.resolve_name('image')), CompressedImage, self.image_callback, queue_size=1, buff_size=self.buff_size)

    def unsubscribe(self):
        self.image_sub.unregister()

    def reconfigure_callback(self, config, level):
        new_config = {}
        if self.old_config:
            new_config = {k: config[k] for k in config if k in self.old_config and config[k] != self.old_config[k]}

        # If we choose config, we sould set DEFAULT to false
        if 'ALL' in new_config and new_config['ALL'] is True:
            self.attributes = ["ALL"]
            for key in config.keys():
                if key in ['ALL', 'groups']:
                    continue
                config[key] = False

        # If we choose DEFAULT, we should set ALL to False
        elif 'DEFAULT' in new_config and new_config['DEFAULT'] is True:
            self.attributes = ["DEFAULT"]
            for key in config.keys():
                if key in ['DEFAULT', 'groups']:
                    continue
                config[key] = False

        else:
            self.attributes = []
            for key in config.keys():
                if key in ['ALL', 'DEFAULT', 'groups']:
                    continue
                if config[key]:
                    self.attributes.append(key)
            # If we want to set individually, remove ALL/DEFAULT
            if self.attributes:
                config['ALL'] = False
                config['DEFAULT'] = False

        self.old_config = config
        return config

    def process_attributes(self, text, img, bbox):
        rospy.logdebug("    {}".format(text))
        if self.use_window:
            cv2.putText(img, text,
                        (bbox.x + bbox.height // 2 + 8, bbox.y - bbox.width // 2 + self.offset), cv2.FONT_HERSHEY_PLAIN,
                        fontScale=1, color=(0, 255, 0), thickness=1, lineType=cv2.LINE_AA)
            self.offset += 16

    @property
    def visualize(self):
        return self.use_window \
            or self.image_pub.get_num_connections() > 0 \
            or self.image_comp_pub.get_num_connections() > 0

    def image_callback(self, image):
        start_time = rospy.Time.now()
        # decode compressed image
        np_arr = np.fromstring(image.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image.format.find("compressed rgb") > -1:
            img = img[:, :, ::-1]

        img_gray = None
        img_width = img.shape[1]
        img_height = img.shape[0]
        visualize = self.visualize
        if visualize:
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img_gray = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)

        _, buf = cv2.imencode('.jpg', img)
        faces = self.rekognition.detect_faces(Image={'Bytes': buf.tobytes()}, Attributes=self.attributes)

        face_msgs = FaceArrayStamped()
        face_msgs.header = image.header
        face_msgs.faces = []

        pose_msgs = PoseArray()
        pose_msgs.header = image.header
        pose_msgs.poses = []

        attributes_msgs = ClassificationResult()
        attributes_msgs.header = image.header
        attributes_msgs.label_names = []
        attributes_msgs.probabilities = []

        landmarks_msgs = PeoplePoseArray()
        landmarks_msgs.header = image.header
        landmarks_msgs.poses = []

        # See https://docs.aws.amazon.com/rekognition/latest/dg/API_DetectFaces.html for detail
        rospy.logdebug("Found {} faces".format(len(faces['FaceDetails'])))
        for face in faces['FaceDetails']:

            # Bounding box of the face
            face_msg = Face()
            bbox_msg = Rect()  # Rect data type, x-y is center point
            if 'BoundingBox' in face:
                top = int(face['BoundingBox']['Top'] * img_height)
                left = int(face['BoundingBox']['Left'] * img_width)
                width = int(face['BoundingBox']['Width'] * img_width)
                height = int(face['BoundingBox']['Height'] * img_height)
                bbox_msg.x = left + width // 2
                bbox_msg.y = top + height // 2
                bbox_msg.width = width
                bbox_msg.height = height

                face_msg.face = bbox_msg

                if visualize:
                    cv2.rectangle(img_gray, (left, top), (left + width, top + height), color=(0, 255, 0), thickness=2)

            # Indicates the location of landmarks on the face.
            if 'Landmarks' in face:
                landmark_msg = PeoplePose()
                landmark_msg.limb_names = []
                landmark_msg.poses = []
                for i in range(len(face['Landmarks'])):
                    landmark = face['Landmarks'][i]
                    px = int(landmark['X'] * img_width)
                    py = int(landmark['Y'] * img_height)

                    landmark_msg.limb_names.append(landmark['Type'])
                    landmark_msg.poses.append(Pose(position=Point(x=px, y=py)))

                    if visualize:
                        cv2.circle(img_gray, (px, py), 1, COLORS[i % (len(COLORS))], thickness=-1)

                landmarks_msgs.poses.append(landmark_msg)

                eye_left_msg = Rect()
                eye_right_msg = Rect()
                landmark = next((x for x in face['Landmarks'] if x['Type'] == 'eyeLeft'), False)
                if landmark:
                    eye_left_msg.x = int(landmark['X'] * img_width)
                    eye_left_msg.y = int(landmark['Y'] * img_height)

                landmark1 = next((x for x in face['Landmarks'] if x['Type'] == 'leftEyeLeft'), False)
                landmark2 = next((x for x in face['Landmarks'] if x['Type'] == 'leftEyeRight'), False)
                if landmark1 and landmark2:
                    eye_left_msg.width = int((landmark2['X'] - landmark1['X']) * img_width)
                landmark1 = next((x for x in face['Landmarks'] if x['Type'] == 'leftEyeUp'), False)
                landmark2 = next((x for x in face['Landmarks'] if x['Type'] == 'leftEyeDown'), False)
                if landmark1 and landmark2:
                    eye_left_msg.height = int((landmark2['Y'] - landmark1['Y']) * img_height)

                landmark = next((x for x in face['Landmarks'] if x['Type'] == 'eyeRight'), False)
                if landmark:
                    eye_right_msg.x = int(landmark['X'] * img_width)
                    eye_right_msg.y = int(landmark['Y'] * img_height)

                landmark1 = next((x for x in face['Landmarks'] if x['Type'] == 'rightEyeLeft'), False)
                landmark2 = next((x for x in face['Landmarks'] if x['Type'] == 'rightEyeRight'), False)
                if landmark1 and landmark2:
                    eye_right_msg.width = int((landmark2['X'] - landmark1['X']) * img_width)
                landmark1 = next((x for x in face['Landmarks'] if x['Type'] == 'rightEyeUp'), False)
                landmark2 = next((x for x in face['Landmarks'] if x['Type'] == 'rightEyeDown'), False)
                if landmark1 and landmark2:
                    eye_right_msg.height = int((landmark2['Y'] - landmark1['Y']) * img_height)

                face_msg.eyes = [eye_left_msg, eye_right_msg]

            # initialize offset in drow text in window
            self.offset = 16

            # Confidence level that the bounding box contains a face.
            if 'Confidence' in face:
                confidence = face['Confidence']
                face_msg.confidence = confidence
                attributes_msgs.label_names.append('confidence')
                attributes_msgs.probabilities.append(confidence)
                self.process_attributes("Confidence : {:.3f}".format(confidence), img_gray, bbox_msg)

            # The estimated age range, in years, for the face. Low represents the lowest estimated age and High represents the highest estimated age.
            if 'AgeRange' in face:
                self.process_attributes("Age Range : {} - {}".format(face['AgeRange']['Low'], face['AgeRange']['High']), img_gray, bbox_msg)

            # Indicates the pose of the face as determined by its pitch, roll, and yaw.
            pose_msg = None
            if 'Pose' in face:
                yaw = face['Pose']['Yaw']
                roll = face['Pose']['Roll']
                pitch = face['Pose']['Pitch']
                q = quaternion_from_euler(roll * math.pi / 180, pitch * math.pi / 180, yaw * math.pi / 180)
                pose_msg = Pose()
                pose_msg.orientation.x = q[0]
                pose_msg.orientation.y = q[1]
                pose_msg.orientation.z = q[2]
                pose_msg.orientation.w = q[3]
                self.process_attributes("Pose : Yaw {:.3f}, Roll {:.3f}, Pitch {:.3f}".format(yaw, roll, pitch), img_gray, bbox_msg)

            # Identifies image brightness and sharpness.
            if 'Quality' in face:
                sharpness = face['Quality']['Sharpness']
                brightness = face['Quality']['Brightness']
                attributes_msgs.label_names.append('sharpness')
                attributes_msgs.probabilities.append(sharpness)
                attributes_msgs.label_names.append('brightness')
                attributes_msgs.probabilities.append(brightness)
                self.process_attributes("Quality : Sharpness {:.3f}, Brightness {:.3f}".format(sharpness, brightness), img_gray, bbox_msg)

            # The emotions that appear to be expressed on the face, and the confidence level in the determination.
            if 'Emotions' in face:
                for emotion in face['Emotions']:
                    if emotion['Confidence'] > 50:
                        face_msg.label += "; {}".format(emotion['Type'])
                    attributes_msgs.label_names.append(emotion['Type'])
                    attributes_msgs.probabilities.append(emotion['Confidence'])
                    self.process_attributes("{}: {:.3f}".format(emotion['Type'], emotion['Confidence']), img_gray, bbox_msg)

            # Other attributes in https://docs.aws.amazon.com/sdkfornet/v3/apidocs/items/Rekognition/TFaceDetail.html
            for key in face.keys():
                if type(face[key]) is dict and 'Confidence' in face[key] and 'Value' in face[key]:
                    if face[key]['Value'] is True:
                        face_msg.label += "; {}".format(key)
                    if not face[key]['Value'] in [True, False]:
                        face_msg.label += "; {}".format(face[key]['Value'])
                    if face[key]['Value'] is True:
                        attributes_msgs.label_names.append(key)
                        attributes_msgs.probabilities.append(face[key]['Confidence'])
                    elif face[key]['Value'] is False:  # If attributes is false, then we use 100-confidnece
                        attributes_msgs.label_names.append(key)
                        attributes_msgs.probabilities.append(100 - face[key]['Confidence'])
                    else:
                        attributes_msgs.label_names.append(face[key]['Value'])
                        attributes_msgs.probabilities.append(face[key]['Confidence'])
                    self.process_attributes("{} : {} ({:.3f})".format(key, face[key]['Value'], face[key]['Confidence']), img_gray, bbox_msg)

            # Construct face message
            face_msg.label = face_msg.label[2:]  # skip first "; "
            face_msgs.faces.append(face_msg)
            # Construct pose message
            if pose_msgs:
                pose_msgs.poses.append(pose_msg)

        if self.use_window:
            cv2.imshow(image._connection_header['topic'], img_gray)
            cv2.waitKey(1)

        # is always_publish is False, and face is not detected , do not publish any results
        if not self.always_publish and len(faces['FaceDetails']) <= 0:
            # debug info
            rospy.loginfo("processing time {}".format((rospy.Time.now() - start_time).to_sec()))
            return

        if self.image_pub.get_num_connections() > 0:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(
                img_gray, encoding='bgr8'))

        if self.image_comp_pub.get_num_connections() > 0:
            msg = CompressedImage()
            msg.header = image.header
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', img_gray)[1]).tostring()
            self.image_comp_pub.publish(msg)

        if self.orig_image_pub.get_num_connections() > 0:
            self.orig_image_pub.publish(image)

        self.faces_pub.publish(face_msgs)
        self.poses_pub.publish(pose_msgs)
        self.attributes_pub.publish(attributes_msgs)
        self.landmarks_pub.publish(landmarks_msgs)

        # debug info
        rospy.logdebug("processing time {} on message taken at {} sec ago".format(
            (rospy.Time.now() - start_time).to_sec(),
            (rospy.Time.now() - image.header.stamp).to_sec()))


if __name__ == '__main__':
    rospy.init_node('aws_detect_faces')
    rospy.loginfo("ROS node initialized as {}".format(rospy.get_name()))
    detect_faces = DetectFaces()
    rospy.spin()
