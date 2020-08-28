#!/usr/bin/env python

import numpy as np

import cv_bridge
from jsk_recognition_utils.depth import split_fore_background
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import RectArray, Rect
import message_filters
import rospy
from sensor_msgs.msg import Image

class Block():
    def __init__(self, rect):
        self.rect = rect
        self.used = False
    def max_cmp(self, another):
        return max(self.rect.width, self.rect.height) - max(another.rect.width, another.rect.height)
    def min_cmp(self, another):
        return min(self.rect.width, self.rect.height) - min(another.rect.width, another.rect.height)
    def height_cmp(self, another):
        return self.rect.height - another.rect.height
    def width_cmp(self, another):
        return self.rect.width - another.rect.width
    def __cmp__(self, another): # python2
        return self.max_cmp(another) or self.min_cmp(another) or self.height_cmp(another) or self.width_cmp(another) or 0
        
class Packer():
    def __init__(self):
        pass
    def fit(self, blocks):
        if len(blocks) == 0:
            return
        self.root = Block(Rect(x=0, y=0, width=blocks[0].rect.width, height=blocks[0].rect.height))
        for block in blocks:
            node = self.find_node(self.root, block.rect.width, block.rect.height)
            if node:
                block.fit_location = self.split(node, block.rect.width, block.rect.height)
            else:
                block.fit_location = self.grow_node(block.rect.width, block.rect.height)
    def find_node(self, root, w, h):
        if root.used:
            return self.find_node(root.right, w, h) or self.find_node(root.down, w, h)
        elif w <= root.rect.width and h <= root.rect.height:
            return root
        else:
            return False
    def split(self, node, w, h):
        node.used = True
        node.down = Block(Rect(x=node.rect.x, y=node.rect.y + h, width=w, height=node.rect.height - h))
        node.right = Block(Rect(x=node.rect.x + w, y=node.rect.y, width=node.rect.width - w, height=h))
        return node
    def grow_node(self, w, h):
        can_grow_down = w <= self.root.rect.width
        can_grow_right = h <= self.root.rect.height
        should_grow_right = can_grow_right and self.root.rect.height >= self.root.rect.width + w
        should_grow_down = can_grow_down and self.root.rect.width >= self.root.rect.height + h
        if should_grow_right:
            return self.grow_right(w, h)
        elif should_grow_down:
            return self.grow_down(w, h)
        elif can_grow_right:
            return self.grow_right(w, h)
        elif can_grow_down:
            return self.grow_down(w, h)
        else:
            return None
    def grow_right(self, w, h):
        prev_root = self.root
        self.root = Block(Rect(x=0, y=0, width=self.root.rect.width + w, height=self.root.rect.height))
        self.root.down = prev_root
        self.root.right = Block(Rect(x=prev_root.rect.width, y=0, width=w, height=prev_root.rect.height))
        self.root.used = True
        node = self.find_node(self.root, w, h)
        if node:
            return self.split(node, w, h)
        else:
            return None
    def grow_down(self, w, h):
        prev_root = self.root
        self.root = Block(Rect(x=0, y=0, width=self.root.rect.width, height=self.root.rect.height + h))
        self.root.right = prev_root
        self.root.down = Block(Rect(x=0, y=prev_root.rect.height, width=prev_root.rect.width, height=h))
        self.root.used = True
        node = self.find_node(self.root, w, h)
        if node:
            return self.split(node, w, h)
        else:
            return None

# def hoge(msg):
#     print "~input", msg.header.stamp.to_sec()
# def fuga(msg):
#     print "~input/rect_array", msg.header.stamp.to_sec()
        
class BinPack(ConnectionBasedTransport):
    def __init__(self):
        super(BinPack, self).__init__()
        self.pub_ = self.advertise('~output', Image, queue_size=10)
    def subscribe(self):
        self.sub_ = message_filters.Subscriber('~input', Image, queue_size=1)
        self.sub_rects_ = message_filters.Subscriber('~input/rect_array', RectArray, queue_size=1)
        # self.sub_.registerCallback(hoge)
        # self.sub_rects_.registerCallback(fuga)
        if rospy.get_param('~approximate_sync', False):
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.sub_, self.sub_rects_], queue_size=100, slop=.1)
        else:
            self.sync = message_filters.TimeSynchronizer(
                [self.sub_, self.sub_rects_], queue_size=100)
        self.sync.registerCallback(self._apply)
    def unsubscribe(self):
        self.sub_.sub.unregister()
        self.sub_rects_.sub.unregister()
    def _apply(self, img_msg, rects):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(img_msg)
        root_rect, blocks = self.bin_pack(rects)
        output_img = np.zeros((root_rect.height, root_rect.width, img.shape[2]), dtype=img.dtype)
        for block in blocks:
            location = block.fit_location
            output_img[location.rect.y:location.rect.y + block.rect.height,
                       location.rect.x:location.rect.x + block.rect.width] = img[block.rect.y:block.rect.y+block.rect.height, block.rect.x:block.rect.x + block.rect.width]
        output_img_msg = bridge.cv2_to_imgmsg(output_img, encoding=img_msg.encoding)
        output_img_msg.header = img_msg.header
        self.pub_.publish(output_img_msg)
    def bin_pack(self, rects):
        # Solve 2D BinPacking problem
        packer = Packer()
        blocks = [Block(rect) for rect in rects.rects]
        # sort by some algorithms
        #blocks.sort(key=lambda x: x.rect.width*x.rect.height, reverse=True)
        blocks.sort(reverse=True)
        packer.fit(blocks)
        print((packer.root.rect.width, packer.root.rect.height))
        return (packer.root.rect, blocks)
        # for block in blocks:
        #     if block.fit_location:
        #         print (block.fit_location.rect.x, block.fit_location.rect.y, block.fit_location.rect.width, block.fit_location.rect.height)
        #         print " --", (block.rect.x, block.rect.y, block.rect.width, block.rect.height)
        
        
if __name__ == '__main__':
    rospy.init_node('binpack')
    bin_pack = BinPack()
    rospy.spin()
