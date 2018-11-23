import os
import os.path as osp

import chainer
import cv2
import numpy as np


class SemanticSegmentationDataset(chainer.dataset.DatasetMixin):

    def __init__(self, root_dir):
        self.root_dir = root_dir

        class_names_path = osp.join(root_dir, 'class_names.txt')
        with open(class_names_path, 'r') as f:
            class_names = f.readlines()
        self.class_names = [name.rstrip() for name in class_names]

        self._images = []
        self._labels = []
        images_dir = osp.join(root_dir, 'JPEGImages')
        labels_dir = osp.join(root_dir, 'SegmentationClass')
        for image_ in sorted(os.listdir(images_dir)):
            image_path = osp.join(images_dir, image_)
            basename = image_.rstrip('.jpg')
            label_path = osp.join(labels_dir, basename + '.npy')
            self._images.append(image_path)
            self._labels.append(label_path)

    def __len__(self):
        return len(self._images)

    def get_example(self, i):
        image_path = self._images[i]
        label_path = self._labels[i]

        image = cv2.imread(image_path)
        assert image.dtype == np.uint8
        assert image.ndim == 3

        label = np.load(label_path)
        assert label.dtype == np.int32
        assert label.ndim == 2

        return image, label
