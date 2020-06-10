from __future__ import print_function

import os
import os.path as osp

from chainercv.chainer_experimental.datasets.sliceable import GetterDataset
import cv2
import numpy as np

import xml.etree.ElementTree as ET

class BboxDetectionDataset(GetterDataset):

    def __init__(self, root_dir):
        super(BboxDetectionDataset, self).__init__()
        self.root_dir = root_dir

        class_names_path = osp.join(root_dir, 'class_names.txt')
        with open(class_names_path, 'r') as f:
            class_names = f.readlines()
        self.class_names = [name.rstrip() for name in class_names]
        self.fg_class_names = np.array(self.class_names[1:])

        self._imgs = []
        self._annotations = []

        imgs_dir = osp.join(root_dir, 'JPEGImages')
        annotation_dir = osp.join(root_dir, 'Annotations')

        for img_ in sorted(os.listdir(imgs_dir)):
            img_path = osp.join(imgs_dir, img_)
            basename = img_.rstrip('.jpg')
            annotation_path = osp.join(annotation_dir, basename + '.xml')
            self._imgs.append(img_path)
            self._annotations.append(annotation_path)

        self.add_getter(('img', 'bbox', 'label'), self._get_example)

    def __len__(self):
        return len(self._imgs)

    def _get_example(self, i):
        img_path = self._imgs[i]
        annotation_path = self._annotations[i]

        img = cv2.imread(img_path)
        assert img.dtype == np.uint8
        assert img.ndim == 3

        label = []
        bbox = []

        tree = ET.parse(annotation_path)
        root = tree.getroot()

        for obj in root.findall("object"):
            label.append(np.where(self.fg_class_names == obj.find("name").text)[0][0])
            bbox.append([obj.find("bndbox").find("ymin").text,
                         obj.find("bndbox").find("xmin").text,
                         obj.find("bndbox").find("ymax").text,
                         obj.find("bndbox").find("xmax").text])

        label = np.array(label, dtype=np.int32)
        bbox = np.array(bbox, dtype=np.float32)

        img = img[:, :, ::-1]
        img = img.transpose((2, 0, 1))
        return img, bbox, label ## this is input
