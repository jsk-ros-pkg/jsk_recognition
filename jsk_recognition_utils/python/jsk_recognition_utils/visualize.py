#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import math

import cv2
import numpy as np
import PIL

from std_msgs.msg import ColorRGBA


def centerize(src, dst_shape):
    """Centerize image for specified image size

    @param src: image to centerize
    @param dst_shape: image shape (height, width) or (height, width, channel)
    """
    if src.shape[:2] == dst_shape[:2]:
        return src
    centerized = np.zeros(dst_shape, dtype=src.dtype)
    pad_vertical, pad_horizontal = 0, 0
    h, w = src.shape[:2]
    dst_h, dst_w = dst_shape[:2]
    if h < dst_h:
        pad_vertical = (dst_h - h) // 2
    if w < dst_w:
        pad_horizontal = (dst_w - w) // 2
    centerized[pad_vertical:pad_vertical+h,
               pad_horizontal:pad_horizontal+w] = src
    return centerized


def _tile_images(imgs, tile_shape):
    """Concatenate images whose sizes are same.

    @param imgs: image list which should be concatenated
    @param tile_shape: shape for which images should be concatenated
    """
    x_num, y_num = tile_shape
    concatenated_image = None
    for y in range(y_num):
        row_image = None
        for x in range(x_num):
            i = x + y * x_num
            if i >= len(imgs):
                img = np.zeros(imgs[0].shape, dtype=np.uint8)
            else:
                img = imgs[i]
            if row_image is None:
                row_image = img
            else:
                row_image = cv2.hconcat([row_image, img])
        if concatenated_image is None:
            concatenated_image = row_image
        else:
            concatenated_image = cv2.vconcat([concatenated_image, row_image])
    return concatenated_image


def get_tile_image(imgs, tile_shape=None):
    """Concatenate images whose sizes are different.

    @param imgs: image list which should be concatenated
    @param tile_shape: shape for which images should be concatenated
    """
    def get_tile_shape(img_num):
        x_num = 0
        y_num = int(math.sqrt(img_num))
        while x_num * y_num < img_num:
            x_num += 1
        return x_num, y_num

    if tile_shape is None:
        tile_shape = get_tile_shape(len(imgs))

    # get max tile size to which each image should be resized
    max_height, max_width = np.inf, np.inf
    for img in imgs:
        max_height = min([max_height, img.shape[0]])
        max_width = min([max_width, img.shape[1]])

    # resize and concatenate images
    for i, img in enumerate(imgs):
        h, w = img.shape[:2]
        h_scale, w_scale = max_height / h, max_width / w
        scale = min([h_scale, w_scale])
        h, w = int(scale * h), int(scale * w)
        img = cv2.resize(img, (w, h))
        img = centerize(img, (max_height, max_width, 3))
        imgs[i] = img
    return _tile_images(imgs, tile_shape)


def color_category20(i):
    colors = (0x1f77b4,
              0xaec7e8,
              0xff7f0e,
              0xffbb78,
              0x2ca02c,
              0x98df8a,
              0xd62728,
              0xff9896,
              0x9467bd,
              0xc5b0d5,
              0x8c564b,
              0xc49c94,
              0xe377c2,
              0xf7b6d2,
              0x7f7f7f,
              0xc7c7c7,
              0xbcbd22,
              0xdbdb8d,
              0x17becf,
              0x9edae5)
    c = colors[i % 20]
    return ColorRGBA(r=(c >> 16) / 255.0, g=((c >> 8) & 255) / 255.0, b=(c & 255) / 255.0, a=1.0)

def color_category10(i):
    colors = (0x1f77b4,
              0xff7f0e,
              0x2ca02c,
              0xd62728,
              0x9467bd,
              0x8c564b,
              0xe377c2,
              0x7f7f7f,
              0xbcbd22,
              0x17becf)
    c = colors[i % 10]
    return ColorRGBA(r=(c >> 16) / 255.0, g=((c >> 8) & 255) / 255.0, b=(c & 255) / 255.0, a=1.0)
