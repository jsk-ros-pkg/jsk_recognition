from __future__ import print_function

import os
import os.path as osp

from chainercv.chainer_experimental.datasets.sliceable import GetterDataset
import cv2
import numpy as np


class DetectionDataset(GetterDataset):

    def __init__(self, root_dir):
        super(DetectionDataset, self).__init__()
        self.root_dir = root_dir

        class_names_path = osp.join(root_dir, 'class_names.txt')
        with open(class_names_path, 'r') as f:
            class_names = f.readlines()
        self.class_names = [name.rstrip() for name in class_names]
        self.fg_class_names = self.class_names[1:]

        self._imgs = []
        self._class_labels = []
        self._instance_labels = []
        imgs_dir = osp.join(root_dir, 'JPEGImages')
        class_labels_dir = osp.join(root_dir, 'SegmentationClass')
        instance_labels_dir = osp.join(root_dir, 'SegmentationObject')
        for img_ in sorted(os.listdir(imgs_dir)):
            img_path = osp.join(imgs_dir, img_)
            basename = img_.rstrip('.jpg')
            class_label_path = osp.join(
                class_labels_dir, basename + '.npy')
            instance_label_path = osp.join(
                instance_labels_dir, basename + '.npy')
            self._imgs.append(img_path)
            self._class_labels.append(class_label_path)
            self._instance_labels.append(instance_label_path)
        self.add_getter(('img', 'bbox', 'label'), self._get_example)

    def __len__(self):
        return len(self._imgs)

    def _get_example(self, i):
        img_path = self._imgs[i]
        class_label_path = self._class_labels[i]
        instance_label_path = self._instance_labels[i]

        img = cv2.imread(img_path)
        assert img.dtype == np.uint8
        assert img.ndim == 3

        class_label = np.load(class_label_path)
        assert class_label.dtype == np.int32
        assert class_label.ndim == 2

        instance_label = np.load(instance_label_path)
        assert instance_label.dtype == np.int32
        assert instance_label.ndim == 2

        assert img.shape[:2] == class_label.shape == instance_label.shape

        # instance id 0 is '_background_' and should be ignored.
        R = np.max(instance_label)
        label = np.zeros((R, ), dtype=np.int32)
        bbox = np.zeros((R, 4), dtype=np.float32)
        for inst_lbl in range(R):
            if inst_lbl == 0:
                continue
            inst_mask = instance_label == inst_lbl
            cls_lbl = np.argmax(np.bincount(class_label[inst_mask]))
            label[inst_lbl] = cls_lbl - 1
            yind, xind = np.where(inst_mask)
            ymin = yind.min()
            ymax = yind.max() + 1
            xmin = xind.min()
            xmax = xind.max() + 1
            bbox[inst_lbl] = np.array(
                [ymin, xmin, ymax, xmax], dtype=np.float32)
        img = img[:, :, ::-1]
        img = img.transpose((2, 0, 1))
        return img, bbox, label
