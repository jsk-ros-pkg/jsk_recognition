#!/usr/bin/env python
# -*- coding: utf-8 -*-

import struct

import cv2
import numpy as np
from skimage.segmentation import slic
from skimage.feature import peak_local_max
from skimage.morphology import binary_closing
import sensor_msgs.msg

from jsk_recognition_utils.mask import descent_closing


def split_fore_background(depth_img, footprint=None):
    if footprint is None:
        footprint = np.ones((3, 3))
    segments = slic(depth_img)

    local_maxi = peak_local_max(
        depth_img, labels=segments, footprint=footprint, indices=False)

    fg_mask = descent_closing(local_maxi, init_selem=np.ones((3, 3)), n_times=6)
    bg_mask = ~fg_mask
    return fg_mask, bg_mask


def depth_to_compressed_depth(depth, depth_max=None, depth_quantization=100,
                              encoding='32FC1'):
    if depth_max is None:
        depth_max = depth.max() + 1.0

    # compressed format is separated by ';'.
    # https://github.com/ros-perception/image_transport_plugins/blob/f0afd122ed9a66ff3362dc7937e6d465e3c3ccf7/compressed_depth_image_transport/src/codec.cpp#L234  # NOQA
    compressed_msg = sensor_msgs.msg.CompressedImage()
    compressed_msg.format = '{}; compressedDepth png'.format(
        encoding)

    if encoding == '32FC1':
        depth_quant_a = depth_quantization * (depth_quantization + 1.0)
        depth_quant_b = 1.0 - depth_quant_a / depth_max
        inv_depth_img = np.zeros_like(depth, dtype=np.uint16)
        target_pixel = np.logical_and(depth_max > depth, depth > 0)
        inv_depth_img[target_pixel] = depth_quant_a / \
            depth[target_pixel] + depth_quant_b

        compressed_msg.data = struct.pack(
            'iff', 0, depth_quant_a, depth_quant_b)
        compressed_msg.data += np.array(
            cv2.imencode('.png', inv_depth_img)[1]).tostring()
    else:
        raise NotImplementedError("Not supported compressedDepth encoding {}"
                                  .format(encoding))
    return compressed_msg
