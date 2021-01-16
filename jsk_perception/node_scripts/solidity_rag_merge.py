#!/usr/bin/env python
# -*- coding: utf-8 -*-

from distutils.version import LooseVersion
import sys

import matplotlib
matplotlib.use('Agg')  # NOQA
import matplotlib.pyplot as plt
import networkx
import numpy as np
import scipy.ndimage as ndi
import skimage
from skimage.color import gray2rgb
from skimage.color import label2rgb
from skimage.future.graph import merge_hierarchical
from skimage.future.graph import RAG
from skimage.future.graph.rag import _add_edge_filter
from skimage.morphology.convex_hull import convex_hull_image
from skimage.segmentation import slic
from skimage.util import img_as_uint

import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
from jsk_topic_tools import warn_no_remap
import message_filters
import rospy
from sensor_msgs.msg import Image

if LooseVersion(skimage.__version__) >= '0.13.0':
    from skimage.future.graph import show_rag
else:
    from skimage.future.graph import draw_rag


###############################################################################
# rag function
###############################################################################

def rag_solidity(labels, connectivity=2):

    graph = RAG()

    # The footprint is constructed in such a way that the first
    # element in the array being passed to _add_edge_filter is
    # the central value.
    fp = ndi.generate_binary_structure(labels.ndim, connectivity)
    for d in range(fp.ndim):
        fp = fp.swapaxes(0, d)
        fp[0, ...] = 0
        fp = fp.swapaxes(0, d)

    # For example
    # if labels.ndim = 2 and connectivity = 1
    # fp = [[0,0,0],
    #       [0,1,1],
    #       [0,1,0]]
    #
    # if labels.ndim = 2 and connectivity = 2
    # fp = [[0,0,0],
    #       [0,1,1],
    #       [0,1,1]]

    ndi.generic_filter(
        labels,
        function=_add_edge_filter,
        footprint=fp,
        mode='nearest',
        output=np.zeros(labels.shape, dtype=np.uint8),
        extra_arguments=(graph,))

    # remove bg_label
    # graph.remove_node(-1)
    graph.remove_node(0)

    for n in graph:
        mask = (labels == n)
        solidity = 1. * mask.sum() / convex_hull_image(mask).sum()
        graph.node[n].update({'labels': [n],
                              'solidity': solidity,
                              'mask': mask})

    if LooseVersion(networkx.__version__) >= '2':
        edges_iter = graph.edges(data=True)
    else:
        edges_iter = graph.edges_iter(data=True)
    for x, y, d in edges_iter:
        new_mask = np.logical_or(graph.node[x]['mask'], graph.node[y]['mask'])
        new_solidity = 1. * new_mask.sum() / convex_hull_image(new_mask).sum()
        org_solidity = np.mean([graph.node[x]['solidity'],
                                graph.node[y]['solidity']])
        d['weight'] = org_solidity / new_solidity

    return graph


###############################################################################
# rag merging functions
###############################################################################

def _solidity_weight_func(graph, src, dst, n):
    """Callback to handle merging nodes by recomputing solidity."""
    org_solidity = np.mean([graph.node[src]['solidity'],
                            graph.node[dst]['solidity']])
    new_mask1 = np.logical_or(graph.node[src]['mask'], graph.node[n]['mask'])
    new_mask2 = np.logical_or(graph.node[dst]['mask'], graph.node[n]['mask'])
    new_solidity1 = 1. * new_mask1.sum() / convex_hull_image(new_mask1).sum()
    new_solidity2 = 1. * new_mask2.sum() / convex_hull_image(new_mask2).sum()
    weight1 = org_solidity / new_solidity1
    weight2 = org_solidity / new_solidity2
    return {'weight': min([weight1, weight2])}


def _solidity_merge_func(graph, src, dst):
    """Callback called before merging two nodes of a solidity graph."""
    new_mask = np.logical_or(graph.node[src]['mask'], graph.node[dst]['mask'])
    graph.node[dst]['mask'] = new_mask
    graph.node[dst]['solidity'] = \
        1. * np.sum(new_mask) / np.sum(convex_hull_image(new_mask))


###############################################################################
# utils
###############################################################################

def masked_slic(img, mask, n_segments, compactness):
    labels = slic(img, n_segments=n_segments, compactness=compactness)
    labels += 1
    n_labels = len(np.unique(labels))
    try:
        mask = ndi.binary_closing(mask, structure=np.ones((3, 3)), iterations=1)
    except IndexError as e:
        rospy.logerr(e)
        return
    labels[mask == 0] = 0  # set bg_label
    if len(np.unique(labels)) < n_labels - 2:
        sys.stderr.write('WARNING: number of label differs after masking.'
                         ' Maybe this is not good for RAG construction.\n')
    return labels


def closed_mask_roi(mask):
    closed_mask = ndi.binary_closing(
        mask, structure=np.ones((3, 3)), iterations=8)
    roi = ndi.find_objects(closed_mask, max_label=1)[0]
    return roi


###############################################################################
# ros node
###############################################################################

class SolidityRagMerge(ConnectionBasedTransport):

    def __init__(self):
        super(SolidityRagMerge, self).__init__()
        self.pub = self.advertise('~output', Image, queue_size=5)
        self.is_debugging = rospy.get_param('~debug', True)
        if self.is_debugging:
            self.pub_rag = self.advertise('~debug/rag', Image, queue_size=5)
            self.pub_slic = self.advertise('~debug/slic', Image, queue_size=5)
            self.pub_label = self.advertise('~debug/label_viz', Image,
                                            queue_size=5)

    def subscribe(self):
        self.sub = message_filters.Subscriber('~input', Image)
        self.sub_mask = message_filters.Subscriber('~input/mask', Image)
        self.use_async = rospy.get_param('~approximate_sync', False)
        rospy.loginfo('~approximate_sync: {}'.format(self.use_async))
        if self.use_async:
            sync = message_filters.ApproximateTimeSynchronizer(
                [self.sub, self.sub_mask], queue_size=1000, slop=0.1)
        else:
            sync = message_filters.TimeSynchronizer(
                [self.sub, self.sub_mask], queue_size=1000)
        sync.registerCallback(self._apply)
        warn_no_remap('~input', '~input/mask')

    def unsubscribe(self):
        self.sub.unregister()
        self.sub_mask.unregister()

    def _apply(self, imgmsg, maskmsg):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg)
        if img.ndim == 2:
            img = gray2rgb(img)
        mask = bridge.imgmsg_to_cv2(maskmsg, desired_encoding='mono8')
        mask = mask.reshape(mask.shape[:2])
        # compute label
        roi = closed_mask_roi(mask)
        roi_labels = masked_slic(img=img[roi], mask=mask[roi],
                                 n_segments=20, compactness=30)
        if roi_labels is None:
            return
        labels = np.zeros(mask.shape, dtype=np.int32)
        # labels.fill(-1)  # set bg_label
        labels[roi] = roi_labels
        if self.is_debugging:
            # publish debug slic label
            slic_labelmsg = bridge.cv2_to_imgmsg(labels)
            slic_labelmsg.header = imgmsg.header
            self.pub_slic.publish(slic_labelmsg)
        # compute rag
        g = rag_solidity(labels, connectivity=2)
        if self.is_debugging:
            # publish debug rag drawn image
            if LooseVersion(skimage.__version__) >= '0.13.0':
                fig, ax = plt.subplots(
                    figsize=(img.shape[1] * 0.01, img.shape[0] * 0.01))
                show_rag(labels, g, img, ax=ax)
                ax.axis('off')
                plt.subplots_adjust(0, 0, 1, 1)
                fig.canvas.draw()
                w, h = fig.canvas.get_width_height()
                rag_img = np.fromstring(
                    fig.canvas.tostring_rgb(), dtype=np.uint8)
                rag_img.shape = (h, w, 3)
                plt.close()
            else:
                rag_img = draw_rag(labels, g, img)
                rag_img = img_as_uint(rag_img)
            rag_imgmsg = bridge.cv2_to_imgmsg(
                rag_img.astype(np.uint8), encoding='rgb8')
            rag_imgmsg.header = imgmsg.header
            self.pub_rag.publish(rag_imgmsg)
        # merge rag with solidity
        merged_labels = merge_hierarchical(
            labels, g, thresh=1, rag_copy=False,
            in_place_merge=True,
            merge_func=_solidity_merge_func,
            weight_func=_solidity_weight_func)
        merged_labels += 1
        merged_labels[mask == 0] = 0
        merged_labelmsg = bridge.cv2_to_imgmsg(merged_labels.astype(np.int32))
        merged_labelmsg.header = imgmsg.header
        self.pub.publish(merged_labelmsg)
        if self.is_debugging:
            out = label2rgb(merged_labels, img)
            out = (out * 255).astype(np.uint8)
            out_msg = bridge.cv2_to_imgmsg(out, encoding='rgb8')
            out_msg.header = imgmsg.header
            self.pub_label.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('solidity_rag_merge')
    SolidityRagMerge()
    rospy.spin()
