#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Create posedetection_msgs/Feature0D dataset from assigned images.

Input:

    container_path/
        category_1_folder/
            file_1
            file_2
            ...
            file_42
        category_2_folder/
            file_43
            file_44
            ...

Output:

    feature0d_dataset.pkl.gz

"""
import os
import sys
import gzip
import cPickle as pickle
import argparse
import collections

import cv2
import numpy as np
from sklearn.datasets.base import Bunch
from sklearn.datasets import load_files

import rospy
import cv_bridge
from posedetection_msgs.srv import Feature0DDetect, Feature0DDetectRequest


def create_feature0d_dataset():
    rospy.init_node('create_feature0d_dataset')

    parser = argparse.ArgumentParser()
    parser.add_argument('container_path', help='image data container path')
    parser.add_argument('-O', '--output', default='feature0d_dataset.pkl.gz',
        help='output filename (default: feature0d_dataset.pkl.gz')
    args = parser.parse_args(rospy.myargv(sys.argv[1:]))

    # See: http://scikit-learn.org/stable/modules/generated/sklearn.datasets.load_files.html
    bunch_files = load_files(container_path=args.container_path,
                             description='image data',
                             shuffle=False,
                             load_content=False)

    # set client for feature0d detection
    client = rospy.ServiceProxy('Feature0DDetect', Feature0DDetect, persistent=True)

    # extract and save features of train images
    bridge = cv_bridge.CvBridge()
    targets, pos, scales, ori, desc = [], [], [], [], []
    for i, (filename, target) in enumerate(zip(bunch_files.filenames,
                                               bunch_files.target)):
        rospy.loginfo(filename)
        targets.append(target)
        # extract feature
        img = cv2.imread(filename)
        imgmsg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
        client.wait_for_service(timeout=10)
        feature0d = client.call(Feature0DDetectRequest(image=imgmsg))
        # save feature data
        pos.append(np.array(feature0d.features.positions))
        scales.append(np.array(feature0d.features.scales))
        ori.append(np.array(feature0d.features.orientations))
        desc.append(np.array(feature0d.features.descriptors))

    dataset = Bunch(target=np.array(targets),
                    target_names=bunch_files.target_names,
                    positions=pos,
                    scales=scales,
                    orientations=ori,
                    descriptors=desc)

    # save features
    print('saving feature0d data')
    with gzip.open(args.output, 'wb') as f:
        pickle.dump(dataset, f)
    rospy.loginfo('finished')


if __name__ == '__main__':
    create_feature0d_dataset()
