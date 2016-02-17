#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import cv2
import numpy as np
import siftfastpy


def get_sift_keypoints(img):
    """Get sift keypoints from image
    Parameters
    ----------
    img: array-like
        Input image of shape ``(M, N)`` from which we get sift keypoints

    Returns
    -------
    frames: numpy.ndarray
        each row has [col, row, orientation, scale] in this order

    desc: numpy.ndarray
        descriptors of each frame
    """
    if type(img) is not np.ndarray:
        img = np.array(img)
    if len(img.shape) != 2:
        raise ValueError('image should be 2d array: {}'.format(img.shape))
    siftimg = siftfastpy.Image(img.shape[1], img.shape[0])
    siftimg.SetData(img)
    frames, desc = siftfastpy.GetKeypoints(siftimg)
    return frames, desc


def draw_sift_frames(img, frames):
    """
    Parameters
    ----------
    img: array-like
        Gray-scale image
    frames: numpy.ndarray
        each row has [col, row, orientation, scale] in this order
    """
    if len(img.shape) > 2:
        raise ValueError('input image should be gray-scale')

    keypoints = []
    for frame in frames:
        col, row, ori, scale = frame
        angle = ori / math.pi * 180
        kp = cv2.KeyPoint(x=col, y=row, _size=scale, _angle=angle)
        keypoints.append(kp)
    dst = cv2.drawKeypoints(img, keypoints,
                            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    return dst
