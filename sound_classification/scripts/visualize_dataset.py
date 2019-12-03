#!/usr/bin/env python
# -*- coding: utf-8 -*-

# visualize created dataset
# You can view next image by pressing keys ('q' is quit)

import argparse
import cv2
import numpy as np
import os
import os.path as osp
from PIL import Image as Image_
import rospkg


def visualize():
    parser = argparse.ArgumentParser()
    parser.add_argument('type', help='visualize train or test', choices=['train', 'test'])
    args = parser.parse_args()
    rospack = rospkg.RosPack()
    data_dir = osp.join(rospack.get_path('sound_classification'),
                        'train_data', 'dataset')
    print("Press 'q' to exit. Press any other keys to show next image.")
    for f in os.listdir(data_dir):
        if not f.startswith(args.type):
            continue
        img = np.array(Image_.open(osp.join(data_dir, f)))
        img = img[:, :, [2, 1, 0]]  # bgr -> rgb
        cv2.imshow('RGB labeled Image', img)
        print('{}'.format(f))
        key = cv2.waitKey(0)
        if key == ord('q'):
            exit()


if __name__ == '__main__':
    visualize()
