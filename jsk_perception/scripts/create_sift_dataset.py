#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Create sift feature dataset from assigned images.

Input:

    container_folder/
        category_1/
            file_1
            file_2
            ...
            file_42
        category_2/
            file_43
            file_44
            ...

Output:

    container_folder_sift_feature.pkl.gz

"""

import argparse
import gzip
import os
import pickle

import cv2
import imagesift
import numpy as np
from sklearn.datasets.base import Bunch
from sklearn.datasets import load_files


def create_sift_dataset():
    parser = argparse.ArgumentParser()
    parser.add_argument('container_path', help='image data container path')
    parser.add_argument('-O', '--output', default=None, help='output file')
    args = parser.parse_args()

    container_path = args.container_path
    output = (args.output or
              os.path.basename(container_path) + '_sift_feature.pkl.gz')

    # See: http://scikit-learn.org/stable/modules/generated/sklearn.datasets.load_files.html
    bunch_files = load_files(container_path=container_path,
                             description='image data',
                             shuffle=False,
                             load_content=False)

    targets, pos_list, scale_list, ori_list, desc_list = [], [], [], [], []
    for i, (filename, target) in enumerate(zip(bunch_files.filenames,
                                               bunch_files.target)):
        print(('filename: {}, label: {}'.format(filename, target)))
        targets.append(target)
        # extract feature
        img = cv2.imread(filename, 0)
        frames, desc = imagesift.get_sift_keypoints(img)
        # save feature data
        pos_list.append(np.hstack([frames[:, 0], frames[:, 1]]))
        ori_list.append(frames[:, 2])
        scale_list.append(frames[:, 3])
        desc_list.append(desc)

    dataset = Bunch(target=np.array(targets),
                    target_names=bunch_files.target_names,
                    positions=pos_list,
                    scales=scale_list,
                    orientations=ori_list,
                    descriptors=desc_list)

    # save features
    print('saving sift feature dataset')
    with gzip.open(output, 'wb') as f:
        pickle.dump(dataset, f)


if __name__ == '__main__':
    create_sift_dataset()
