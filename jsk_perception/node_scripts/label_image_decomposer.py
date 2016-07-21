#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from skimage.color import gray2rgb
from skimage.color import label2rgb
from skimage.util import img_as_ubyte

import cv_bridge
from jsk_recognition_utils import bounding_rect_of_mask
from jsk_recognition_utils import get_tile_image
from jsk_recognition_utils.color import labelcolormap
from jsk_topic_tools import ConnectionBasedTransport
from jsk_topic_tools import jsk_loginfo
from jsk_topic_tools import warn_no_remap
import matplotlib
matplotlib.use('Agg')  # NOQA
import matplotlib.pyplot as plt
import message_filters
import rospy
from sensor_msgs.msg import Image


class LabelImageDecomposer(ConnectionBasedTransport):

    def __init__(self):
        super(LabelImageDecomposer, self).__init__()
        self.pub_img = self.advertise('~output', Image, queue_size=5)
        self.pub_label_viz = self.advertise('~output/label_viz', Image,
                                            queue_size=5)
        self._label_names = rospy.get_param('~label_names', [])
        # publish masks of fg/bg by decomposing each label
        self._publish_mask = rospy.get_param('~publish_mask', False)
        if self._publish_mask:
            self.pub_fg_mask = self.advertise('~output/fg_mask', Image,
                                              queue_size=5)
            self.pub_bg_mask = self.advertise('~output/bg_mask', Image,
                                                    queue_size=5)
        # publish each region image. this can take time so optional.
        self._publish_tile = rospy.get_param('~publish_tile', False)
        jsk_loginfo('~publish_tile: {}'.format(self._publish_tile))
        if self._publish_tile:
            self.pub_tile = self.advertise('~output/tile', Image, queue_size=5)

    def subscribe(self):
        self.sub_img = message_filters.Subscriber('~input', Image)
        self.sub_label = message_filters.Subscriber('~input/label', Image)
        warn_no_remap('~input', '~input/label')
        use_async = rospy.get_param('~approximate_sync', False)
        queue_size = rospy.get_param('~queue_size', 10)
        jsk_loginfo('~approximate_sync: {}, queue_size: {}'
                    .format(use_async, queue_size))
        if use_async:
            slop = rospy.get_param('~slop', 0.1)
            jsk_loginfo('~slop: {}'.format(slop))
            async = message_filters.ApproximateTimeSynchronizer(
                [self.sub_img, self.sub_label],
                queue_size=queue_size, slop=slop)
            async.registerCallback(self._apply)
            if self._publish_tile:
                async.registerCallback(self._apply_tile)
        else:
            sync = message_filters.TimeSynchronizer(
                [self.sub_img, self.sub_label], queue_size=queue_size)
            sync.registerCallback(self._apply)
            if self._publish_tile:
                sync.registerCallback(self._apply_tile)

    def unsubscribe(self):
        self.sub_img.sub.unregister()
        self.sub_label.sub.unregister()

    def _apply(self, img_msg, label_msg):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(img_msg)
        label_img = bridge.imgmsg_to_cv2(label_msg)
        # publish only valid label region
        applied = img.copy()
        applied[label_img == 0] = 0
        applied_msg = bridge.cv2_to_imgmsg(applied, encoding=img_msg.encoding)
        applied_msg.header = img_msg.header
        self.pub_img.publish(applied_msg)
        # publish visualized label
        if img_msg.encoding in {'16UC1', '32SC1'}:
            # do dynamic scaling to make it look nicely
            min_value, max_value = img.min(), img.max()
            img = (img - min_value) / (max_value - min_value) * 255
            img = gray2rgb(img)

        unique_labels = np.unique(label_img)
        if self._label_names:
            n_label = len(self._label_names)
        else:
            n_label = len(unique_labels)
        cmap = labelcolormap(N=n_label)
        label_img = label_img.copy()
        label_viz = label2rgb(label_img, img, colors=cmap[1:], bg_label=0)
        label_viz = img_as_ubyte(label_viz)

        if self._label_names:
            # plot label titles on image using matplotlib
            import StringIO
            import PIL.Image
            from scipy.misc import fromimage
            plt.subplots_adjust(left=0, right=1, top=1, bottom=0,
                                wspace=0, hspace=0)
            plt.margins(0, 0)
            plt.gca().xaxis.set_major_locator(plt.NullLocator())
            plt.gca().yaxis.set_major_locator(plt.NullLocator())
            plt.axis('off')
            # plot image
            plt.imshow(label_viz)
            # plot legend
            plt_handlers = []
            plt_titles = []
            for label_value in unique_labels:
                if (label_img == label_value).sum() < 0.01 * label_img.size:
                    # Skip label which has small region
                    continue
                fc = cmap[label_value]
                p = plt.Rectangle((0, 0), 1, 1, fc=fc)
                plt_handlers.append(p)
                plt_titles.append(self._label_names[label_value])
            plt.legend(plt_handlers, plt_titles, loc='lower right',
                       framealpha=0.5)
            # convert plotted figure to np.ndarray
            f = StringIO.StringIO()
            plt.savefig(f, bbox_inches='tight', pad_inches=0)
            result_img_pil = PIL.Image.open(f)
            result_img = fromimage(result_img_pil, mode='RGB')
            label_viz = result_img

        label_viz_msg = bridge.cv2_to_imgmsg(label_viz, encoding='rgb8')
        label_viz_msg.header = img_msg.header
        self.pub_label_viz.publish(label_viz_msg)

        # publish mask
        if self._publish_mask:
            bg_mask = (label_img == 0)
            fg_mask = ~bg_mask
            bg_mask = (bg_mask * 255).astype(np.uint8)
            fg_mask = (fg_mask * 255).astype(np.uint8)
            fg_mask_msg = bridge.cv2_to_imgmsg(fg_mask, encoding='mono8')
            fg_mask_msg.header = img_msg.header
            bg_mask_msg = bridge.cv2_to_imgmsg(bg_mask, encoding='mono8')
            bg_mask_msg.header = img_msg.header
            self.pub_fg_mask.publish(fg_mask_msg)
            self.pub_bg_mask.publish(bg_mask_msg)

    def _apply_tile(self, img_msg, label_msg):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(img_msg)
        label_img = bridge.imgmsg_to_cv2(label_msg)

        imgs = []
        labels = np.unique(label_img)
        for label in labels:
            if label == 0:
                # should be skipped 0, because
                # 0 is to label image as black region to mask image
                continue
            img_tmp = img.copy()
            mask = label_img == label
            img_tmp[~mask] = 0
            img_tmp = bounding_rect_of_mask(img_tmp, mask)
            imgs.append(img_tmp)
        tile_img = get_tile_image(imgs)
        tile_msg = bridge.cv2_to_imgmsg(tile_img, encoding='bgr8')
        tile_msg.header = img_msg.header
        self.pub_tile.publish(tile_msg)


if __name__ == '__main__':
    rospy.init_node('label_image_decomposer')
    label_image_decomposer = LabelImageDecomposer()
    rospy.spin()
