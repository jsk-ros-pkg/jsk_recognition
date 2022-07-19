#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os, sys
from threading import Lock

import cv2
import numpy as np

import cv_bridge
from cv_bridge.boost.cv_bridge_boost import getCvType
import dynamic_reconfigure.server
import rospy

from jsk_perception.cfg import ImagePublisherConfig
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image, CompressedImage

import re
from PIL import Image as PIL_Image
from PIL import ExifTags as PIL_ExifTags
from jsk_recognition_msgs.msg import ExifTags, ExifGPSInfo
import imghdr

from jsk_recognition_utils.depth import depth_to_compressed_depth


class ImagePublisher(object):

    def __init__(self):
        self.lock = Lock()
        self.imgmsg = None
        self.encoding = rospy.get_param('~encoding', 'bgr8')
        self.frame_id = rospy.get_param('~frame_id', 'camera')
        self.fovx = rospy.get_param('~fovx', None)
        self.fovy = rospy.get_param('~fovy', None)
        self.publish_exif = rospy.get_param('~publish_exif', False)
        if (self.fovx is None) != (self.fovy is None):
            rospy.logwarn('fovx and fovy should be specified, but '
                          'specified only {}'
                          .format('fovx' if self.fovx else 'fovy'))
        dynamic_reconfigure.server.Server(
            ImagePublisherConfig, self._cb_dyn_reconfig)
        self.pub = rospy.Publisher('~output', Image, queue_size=1)
        self.pub_compressed = rospy.Publisher('{}/compressed'.format(rospy.resolve_name('~output')), CompressedImage, queue_size=1)
        if self.encoding == '32FC1':
            self.pub_compressed = rospy.Publisher(
                '{}/compressedDepth'.format(rospy.resolve_name('~output')),
                CompressedImage, queue_size=1)
        self.publish_info = rospy.get_param('~publish_info', True)
        if self.publish_info:
            self.pub_info = rospy.Publisher(
                '~output/camera_info', CameraInfo, queue_size=1)
        if self.publish_exif:
            self.pub_exif = rospy.Publisher('~exif', ExifTags, queue_size=1)
        rate = rospy.get_param('~rate', 1.)
        rospy.Timer(rospy.Duration(1. / rate), self.publish)

    def _update_exif_data(self, data, tag, value, get = PIL_ExifTags.TAGS.get):
        def encode_value(value, attr, data):
            attr_type = type(attr)
            value_type = type(value)
            if attr_type == list and value_type == tuple:
                for i in range(len(value)):
                    attr[i] = encode_value(value[i], attr[i], data)
                return attr
            else:
                if attr_type == str and value_type == unicode:
                    # if type is unicode, conver to string
                    value = value.encode('ascii', 'ignore')
                if attr_type == int and value_type == str:
                    # binary string to uint8
                    value = map(ord, value)[0]
                elif attr_type == float and value_type == tuple:
                    # float tuple to tuple
                    value = float('{}.{}'.format(value[0], value[1]))
                return value

        string_tag = get(tag, tag)
        if tag == 0x927c:
            pass  # skip MakerNote
        elif type(string_tag) == str:
            decoded_tag = re.sub('([a-z])([A-Z])', '\\1_\\2', string_tag)
            decoded_tag = re.sub('([a-zA-Z])([0-9])', '\\1_\\2', decoded_tag)
            decoded_tag = re.sub('([XYZ])Resolution', '\\1_resolution', decoded_tag)
            decoded_tag = re.sub('([F])Number', '\\1_number', decoded_tag)
            decoded_tag = re.sub('(ISO)', '\\1_', decoded_tag)
            decoded_tag = re.sub('(GPS)', '\\1_', decoded_tag)
            decoded_tag = decoded_tag.lower()
            if decoded_tag == 'gps_info':
                gps_data = ExifGPSInfo()
                for t in value:
                    self._update_exif_data(gps_data, t, value[t], get=PIL_ExifTags.GPSTAGS.get)
                data.gps_info = gps_data
            else:
                try:
                    setattr(data, decoded_tag, encode_value(value, getattr(data, decoded_tag), data))
                except AttributeError as e:
                    rospy.logwarn("Undefiend exif tags [{:x} {}({}) {}]".format(tag, string_tag, decoded_tag, value))
                except Exception as e:
                     rospy.logerr("Wrong exif information {} [{:x} {}({}) {}]".format(e, tag, string_tag, decoded_tag, value))
        else:
            rospy.logwarn("Unknown exif tags [{:x} {} {}]".format(tag, string_tag, value))
        return data

    def _get_exif_data(self, file_name):
        exif_data =ExifTags()
        self.info = PIL_Image.open(file_name)._getexif()
        if self.info:
            for tag, value in self.info.items():
                self._update_exif_data(exif_data, tag, value)
        return exif_data

    def _cb_dyn_reconfig(self, config, level):
        file_name = config['file_name']
        config['file_name'] = os.path.abspath(file_name)
        self.dst_format = imghdr.what(file_name)
        img = cv2.imread(file_name, cv2.IMREAD_UNCHANGED)
        if img is None:
            rospy.logwarn('Could not read image file: {}'.format(file_name))
            with self.lock:
                self.imgmsg = None
        else:
            rospy.loginfo('Read the image file: {}'.format(file_name))
            # when file is gray scale but encoding is not grayscale,
            # load the image again in color
            if (len(img.shape) == 2 and
                getCvType(self.encoding) not in [
                    cv2.CV_8UC1, cv2.CV_16UC1, cv2.CV_32FC1]):
                img = cv2.imread(file_name, cv2.IMREAD_COLOR)
            # read exif
            if self.publish_exif:
                self.exif_data = self._get_exif_data(file_name)
            with self.lock:
                self.imgmsg, self.compmsg = \
                    self.cv2_to_imgmsg(img, self.encoding)
        return config

    def publish(self, event):
        if self.imgmsg is None:
            return
        now = rospy.Time.now()
        # setup ros message and publish
        with self.lock:
            self.imgmsg.header.stamp = \
                self.compmsg.header.stamp = now
            self.imgmsg.header.frame_id = \
                self.compmsg.header.frame_id = self.frame_id
        if self.pub.get_num_connections() > 0:
            self.pub.publish(self.imgmsg)
        if self.pub_compressed.get_num_connections() > 0:
            self.pub_compressed.publish(self.compmsg)
        if self.publish_info:
            info = CameraInfo()
            info.header.stamp = now
            info.header.frame_id = self.frame_id
            info.width = self.imgmsg.width
            info.height = self.imgmsg.height
            if self.fovx is not None and self.fovy is not None:
                fx = self.imgmsg.width / 2.0 / \
                    np.tan(np.deg2rad(self.fovx / 2.0))
                fy = self.imgmsg.height / 2.0 / \
                    np.tan(np.deg2rad(self.fovy / 2.0))
                cx = self.imgmsg.width / 2.0
                cy = self.imgmsg.height / 2.0
                info.K = np.array([fx, 0, cx,
                                   0, fy, cy,
                                   0, 0, 1.0])
                info.P = np.array([fx, 0, cx, 0,
                                   0, fy, cy, 0,
                                   0, 0, 1, 0])
                info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
            self.pub_info.publish(info)
        if self.publish_exif:
            self.pub_exif.publish(self.exif_data)

    def cv2_to_imgmsg(self, img, encoding):
        bridge = cv_bridge.CvBridge()
        # resolve encoding
        cv_type = getCvType(encoding)
        dst_format = self.dst_format
        if cv_type in [cv2.CV_8UC1, cv2.CV_16UC1, cv2.CV_32FC1]:
            # mono8
            if len(img.shape) == 3:
                if img.shape[2] == 4:
                    code = cv2.COLOR_BGRA2GRAY
                else:
                    code = cv2.COLOR_BGR2GRAY
                img = cv2.cvtColor(img, code)
            if cv_type == cv2.CV_16UC1:
                # 16UC1
                # png compression on 16-bit images
                dst_format = 'png'
                img = img.astype(np.float32)
                img = np.clip(img / 255.0 * (2 ** 16 - 1), 0, 2 ** 16 - 1)
                img = img.astype(np.uint16)
            elif cv_type == cv2.CV_32FC1:
                # 32FC1
                img = img.astype(np.float32)
                img /= 255.0
                comp_depth_msg = depth_to_compressed_depth(img)
                return bridge.cv2_to_imgmsg(img, encoding=encoding), comp_depth_msg
            comp_img = img
            target_format = encoding
        elif cv_type == cv2.CV_8UC3 and len(img.shape) == 3:
            # 8UC3
            # BGRA, BGR -> BGR
            img = comp_img = img[:, :, :3]
            target_format = 'bgr8'
            # BGR -> RGB
            if encoding == 'rgb8':
                img = img[:, :, ::-1]
        elif cv_type == cv2.CV_16UC3 and len(img.shape) == 3:
            # png compression on 16-bit images
            dst_format = 'png'
            # 16UC3
            # BGRA, BGR -> BGR
            img = img[:, :, :3]
            # convert to 16UC3 image.
            img = img.astype(np.float32)
            img = np.clip(img / 255.0 * (2 ** 16 - 1), 0, 2 ** 16 - 1)
            img = comp_img = img.astype(np.uint16)
            target_format = 'bgr16'
            # BGR -> RGB
            if encoding == 'rgb16':
                img = img[:, :, ::-1]
        elif (cv_type == cv2.CV_8UC4 and
                len(img.shape) == 3 and img.shape[2] == 4):
            comp_img = img[:, :, :3]
            target_format = 'bgr8'
            # 8UC4
            if encoding == 'rgba8':
                # BGRA -> RGBA
                img = img[:, :, [2, 1, 0, 3]]
        elif (cv_type == cv2.CV_16UC4 and
                len(img.shape) == 3 and img.shape[2] == 4):
            # convert to 16UC4 image.
            img = img.astype(np.float32)
            img = np.clip(img / 255.0 * (2 ** 16 - 1), 0, 2 ** 16 - 1)
            img = img.astype(np.uint16)
            comp_img = img[:, :, :3]
            target_format = 'bgr16'
            if encoding == 'rgba16':
                # BGRA -> RGBA
                img = img[:, :, [2, 1, 0, 3]]
        else:
            rospy.logerr('unsupported encoding: {0}'.format(encoding))
            return
        compmsg = bridge.cv2_to_compressed_imgmsg(comp_img, dst_format=dst_format)
        # compressed format is separated by ';'.
        # https://github.com/ros-perception/image_transport_plugins/blob/f0afd122ed9a66ff3362dc7937e6d465e3c3ccf7/compressed_image_transport/src/compressed_publisher.cpp#L116-L128
        compmsg.format = '{}; {} compressed {}'.format(
            encoding, dst_format, target_format)
        return bridge.cv2_to_imgmsg(img, encoding=encoding), compmsg



if __name__ == '__main__':
    rospy.init_node('image_publisher')
    ImagePublisher()
    rospy.spin()
