from __future__ import division

import os
import os.path as osp

import chainer
import cv2
import imgaug.augmenters as iaa
import numpy as np
import PIL.Image
import rospkg
import skimage.io

import mvtk


class DepthPredictionDataset(chainer.dataset.DatasetMixin):

    class_names = np.array([
        '_background_',
        'mirror',
    ], dtype=np.str)
    class_names.setflags(write=0)

    _files = set([
        'depth.npz',
        'depth_gt.npz',
        'image.png',
        'label.png'
    ])

    rospack = rospkg.RosPack()
    root_dir = osp.join(
        rospack.get_path('jsk_perception'), 'learning_datasets',
        'human_size_mirror_dataset')
    mean_bgr = np.array([104.00698793, 116.66876762, 122.67891434])
    min_value = 0.5
    max_value = 5.0

    def __init__(self, split, aug=False, num_view=1):
        assert split in ['train', 'test']
        self.split = split
        self.aug = aug
        self.num_view = num_view

        self._files_dirs = []
        self._scenes = []
        for date_dir in sorted(os.listdir(self.root_dir)):
            date_dir = osp.join(self.root_dir, date_dir)
            split_dir = osp.join(date_dir, split)
            for files_dir in sorted(os.listdir(split_dir)):
                files_dir = osp.join(split_dir, files_dir)
                files = set(os.listdir(files_dir))
                assert self._files.issubset(files), (
                    'In {}: File set does not match.\n'
                    'Expected: {}\nActual: {}'
                    .format(files_dir, self._files, files))
                self._files_dirs.append(files_dir)

    def __len__(self):
        return len(self._files_dirs)

    def get_example(self, i):
        examples = []

        # Append index == i as first example
        examples.append(self._get_example(self._files_dirs[i], i))

        assert len(examples) == self.num_view

        return examples

    def _get_example(self, files_dir, idx):
        image_file = osp.join(files_dir, 'image.png')
        image = skimage.io.imread(image_file)
        assert image.dtype == np.uint8
        assert image.ndim == 3

        depth_file = osp.join(files_dir, 'depth.npz')
        depth = np.load(depth_file)['arr_0']
        depth[depth == 0] = np.nan
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32)
            depth /= 1000
        depth_keep = ~np.isnan(depth)
        depth[depth_keep] = np.maximum(depth[depth_keep], self.min_value)
        depth[depth_keep] = np.minimum(depth[depth_keep], self.max_value)
        assert depth.dtype == np.float32
        assert depth.ndim == 2

        label_file = osp.join(files_dir, 'label.png')
        with open(label_file, 'r') as f:
            label = np.asarray(PIL.Image.open(f)).astype(np.int32)
        assert label.dtype == np.int32
        assert label.ndim == 2

        depth_gt_file = osp.join(files_dir, 'depth_gt.npz')
        depth_gt = np.load(depth_gt_file)['arr_0']
        depth_gt[depth_gt == 0] = np.nan
        if depth_gt.dtype == np.uint16:
            depth_gt = depth_gt.astype(np.float32)
            depth /= 1000
        depth_gt_keep = ~np.isnan(depth_gt)
        depth_gt[depth_gt_keep] = np.maximum(
            depth_gt[depth_gt_keep], self.min_value)
        depth_gt[depth_gt_keep] = np.minimum(
            depth_gt[depth_gt_keep], self.max_value)
        assert depth_gt.dtype == np.float32
        assert depth_gt.ndim == 2

        # Data augmentation
        if self.aug:
            # 1. Color augmentation
            obj_datum = dict(img=image)
            random_state = np.random.RandomState()

            def st(x):
                return iaa.Sometimes(0.3, x)

            augs = [
                st(iaa.Add([-50, 50], per_channel=True)),
                st(iaa.InColorspace(
                    'HSV', children=iaa.WithChannels(
                        [1, 2], iaa.Multiply([0.5, 2])))),
                st(iaa.GaussianBlur(sigma=[0.0, 1.0])),
                st(iaa.AdditiveGaussianNoise(
                    scale=(0.0, 0.1 * 255), per_channel=True)),
            ]
            obj_datum = next(mvtk.aug.augment_object_data(
                [obj_datum], random_state=random_state, augmentations=augs))
            image = obj_datum['img']

            # 2. Depth noise
            np.random.seed()
            if np.random.uniform() < 0.3:
                noise_rate = np.random.uniform() * 0.25 + 0.05
                depth[
                    np.random.rand(depth.shape[0], depth.shape[1]) < noise_rate
                ] = np.nan

            # 3. Geometric augmentation
            if np.random.uniform() < 0.5:
                image = np.fliplr(image)
                depth = np.fliplr(depth)
                label = np.fliplr(label)
                depth_gt = np.fliplr(depth_gt)
            if np.random.uniform() < 0.5:
                image = np.flipud(image)
                depth = np.flipud(depth)
                label = np.flipud(label)
                depth_gt = np.flipud(depth_gt)
            if np.random.uniform() < 0.5:
                angle = (np.random.uniform() * 180) - 90
                image = self.rotate_image(image, angle, cv2.INTER_LINEAR)
                depth = self.rotate_depth_image(depth, angle, cv2.INTER_LINEAR)
                label = self.rotate_image(label, angle, cv2.INTER_NEAREST)
                depth_gt = self.rotate_depth_image(
                    depth_gt, angle, cv2.INTER_LINEAR)

        return image, depth, label, depth_gt, idx

    def rotate_image(self, in_img, angle, flags=cv2.INTER_LINEAR):
        rot_mat = cv2.getRotationMatrix2D(
            center=(in_img.shape[1] / 2, in_img.shape[0] / 2),
            angle=angle, scale=1)
        rot_img = cv2.warpAffine(
            src=in_img, M=rot_mat,
            dsize=(in_img.shape[1], in_img.shape[0]), flags=flags)
        return rot_img

    def rotate_depth_image(self, in_img, angle, flags=cv2.INTER_LINEAR):
        rot_mat = cv2.getRotationMatrix2D(
            center=(in_img.shape[1] / 2, in_img.shape[0] / 2),
            angle=angle, scale=1)
        ones = np.ones(in_img.shape, dtype=np.int32)
        rot_keep = cv2.warpAffine(
            src=ones, M=rot_mat,
            dsize=(in_img.shape[1], in_img.shape[0]),
            flags=cv2.INTER_NEAREST)
        rot_keep = rot_keep.astype(np.bool)
        rot_img = cv2.warpAffine(
            src=in_img, M=rot_mat,
            dsize=(in_img.shape[1], in_img.shape[0]), flags=flags)
        rot_img[rot_keep == False] = np.nan  # NOQA
        return rot_img

    def visualize(self, index):
        examples = self[index]

        print('[%04d] %s' % (index, '>' * 75))
        print('got example indices: {}'.format(
            [example[4] for example in examples]))
        print('[%04d] %s' % (index, '<' * 75))

        viz = []
        for i, example in enumerate(examples):
            image, depth, label, depth_gt, idx = example
            depth_viz = mvtk.image.colorize_depth(
                depth, min_value=self.min_value, max_value=self.max_value)
            label = mvtk.image.label2rgb(
                label.astype(np.int32), img=image,
                label_names=self.class_names, alpha=0.7)
            depth_gt_viz = mvtk.image.colorize_depth(
                depth_gt, min_value=self.min_value, max_value=self.max_value)
            viz.append(mvtk.image.tile(
                [image, depth_viz, label, depth_gt_viz], (2, 2)))

        viz_all = mvtk.image.tile(viz, (1, len(examples)))

        return mvtk.image.resize(viz_all, size=600 * 600)  # for small window
