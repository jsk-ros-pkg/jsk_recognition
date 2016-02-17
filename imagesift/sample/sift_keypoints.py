#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

import cv2

import rospkg
import imagesift


def main():
    rp = rospkg.RosPack()
    imgpath = os.path.join(rp.get_path('jsk_perception'), 'sample/ros_fuerte.jpg')

    img = cv2.imread(imgpath, 0)  # gray-scale image

    frames, desc = imagesift.get_sift_keypoints(img)

    out = imagesift.draw_sift_frames(img, frames)

    cv2.imshow('sift image', out)
    cv2.waitKey(0)


if __name__ == '__main__':
    main()
