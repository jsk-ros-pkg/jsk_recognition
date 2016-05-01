#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
from scipy.misc import lena

import imagesift


def main():
    img = lena()

    frames, desc = imagesift.get_sift_keypoints(img)

    out = imagesift.draw_sift_frames(img, frames)

    cv2.imshow('sift image', out)
    cv2.waitKey(0)


if __name__ == '__main__':
    main()
