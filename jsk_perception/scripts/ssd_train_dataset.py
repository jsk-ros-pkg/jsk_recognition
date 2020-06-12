#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from __future__ import print_function

import argparse
import copy
import json
import numpy as np
import os
import sys
import yaml

# chainer
import itertools, pkg_resources
from distutils.version import LooseVersion
if LooseVersion(pkg_resources.get_distribution("chainer").version) >= LooseVersion('7.0.0') and \
   sys.version_info.major == 2:
   print('''Please install chainer <= 7.0.0:

    sudo pip install chainer==6.7.0

c.f https://github.com/jsk-ros-pkg/jsk_recognition/pull/2485
''', file=sys.stderr)
   sys.exit(1)
if [p for p in list(itertools.chain(*[pkg_resources.find_distributions(_) for _ in sys.path])) if "cupy-" in p.project_name ] == []:
   print('''Please install CuPy

    sudo pip install cupy-cuda[your cuda version]
i.e.
    sudo pip install cupy-cuda91

''', file=sys.stderr)
   sys.exit(1)
import chainer
from chainer import serializers
from chainer import training
from chainer.datasets import TransformDataset
from chainer.training import extensions
from chainer.optimizer import WeightDecay

# chainercv
from chainercv import transforms
from chainercv.extensions import DetectionVOCEvaluator
from chainercv.links import SSD300
from chainercv.links.model.ssd import GradientScaling
from chainercv.links.model.ssd import multibox_loss
from chainercv.links.model.ssd import random_distort
from chainercv.links.model.ssd import random_crop_with_bbox_constraints
from chainercv.links.model.ssd import resize_with_random_interpolation
from chainercv.utils import read_image


class SSDDataset(chainer.dataset.DatasetMixin):

    def __init__(self, base_dir, label_names):
        self.base_dir = base_dir
        self.label_names = label_names

        self.img_filenames = []
        for name in os.listdir(base_dir):
            # If the file is not an image, ignore the file.
            if os.path.splitext(name)[1] != '.jpg':
                continue
            self.img_filenames.append(os.path.join(base_dir, name))

    def __len__(self):
        return len(self.img_filenames)

    def get_example(self, i):
        img_filename = self.img_filenames[i]
        img = read_image(img_filename)

        anno_filename = os.path.splitext(img_filename)[0] + '__labels.json'

        with open(anno_filename, 'r') as f:
            anno = json.load(f)
        anno = anno['labels']

        bbox = []
        label = []
        for anno_i in anno:
            h = anno_i['size']['y']
            w = anno_i['size']['x']
            center_y = anno_i['centre']['y']
            center_x = anno_i['centre']['x']
            try:
                l = self.label_names.index(anno_i['label_class'])
            except Exception as e:
                print("Failed to index label class: {}".format(anno_i), file=sys.stderr)
                print("image file name: {}".format(img_filename), file=sys.stderr)
                print("annotation file name: {}".format(anno_filename), file=sys.stderr)
                continue
            bbox.append(
                [center_y - h / 2, center_x - w / 2,
                 center_y + h / 2, center_x + w / 2])
            label.append(l)
        return img, np.array(bbox, dtype=np.float32), np.array(label, dtype=np.int32)


class MultiboxTrainChain(chainer.Chain):

    def __init__(self, model, alpha=1, k=3):
        super(MultiboxTrainChain, self).__init__()
        with self.init_scope():
            self.model = model
        self.alpha = alpha
        self.k = k

    def __call__(self, imgs, gt_mb_locs, gt_mb_labs):
        mb_locs, mb_confs = self.model(imgs)
        loc_loss, conf_loss = multibox_loss(
            mb_locs, mb_confs, gt_mb_locs, gt_mb_labs, self.k)
        loss = loc_loss * self.alpha + conf_loss

        chainer.reporter.report(
            {'loss': loss, 'loss/loc': loc_loss, 'loss/conf': conf_loss},
            self)

        return loss


class Transform(object):
    """Class for augumentation"""

    def __init__(self, coder, size, mean):
        # copy to send to cpu
        self.coder = copy.copy(coder)
        self.coder.to_cpu()

        self.size = size
        self.mean = mean

    def __call__(self, in_data):
        img, bbox, label = in_data

        # 1. Color augumentation
        img = random_distort(img)

        # 2. Random expansion
        if np.random.randint(2):
            img, param = transforms.random_expand(
                img, fill=self.mean, return_param=True)
            bbox = transforms.translate_bbox(
                bbox, y_offset=param["y_offset"], x_offset=param["x_offset"])

        # 3. Random cropping
        img, param = random_crop_with_bbox_constraints(
            img, bbox, return_param=True)
        bbox, param = transforms.crop_bbox(
            bbox, y_slice=param["y_slice"], x_slice=param["x_slice"],
            allow_outside_center=False, return_param=True)
        label = label[param["index"]]

        # 4. Resizing with random interpolation
        _, H, W = img.shape
        img = resize_with_random_interpolation(img, (self.size, self.size))
        bbox = transforms.resize_bbox(bbox, (H, W), (self.size, self.size))

        # 5. Transformation for SSD network input
        img -= self.mean
        mb_loc, mb_lab = self.coder.encode(bbox, label)

        return img, mb_loc, mb_lab


if __name__ == '__main__':

    p = argparse.ArgumentParser()
    p.add_argument("label_file", help="path to label file")
    p.add_argument("train", help="path to train dataset directory")
    p.add_argument("--val", help="path to validation dataset directory. If this argument is not specified, train dataset is used with ratio train:val = 8:2.", default=None)
    p.add_argument("--base-model", help="base model name", default="voc0712")
    p.add_argument("--batchsize", "-b", type=int, default=16)
    p.add_argument("--iteration", type=int, default=120000)
    p.add_argument("--gpu", "-g", type=int, default=-1)  # use CPU by default
    p.add_argument("--out", "-o", type=str, default="results")
    p.add_argument("--resume", type=str, default="")
    p.add_argument("--lr", type=float, default=1e-4)
    p.add_argument("--val-iter", type=int, default=100)
    p.add_argument("--log-iter", type=int, default=10)
    p.add_argument("--model-iter", type=int, default=200)

    args = p.parse_args()

    # load label file
    with open(args.label_file, "r") as f:
        label_names = tuple(yaml.load(f))

    print("Loaded %d labels" % len(label_names))

    if args.val is None:
        dataset = SSDDataset(args.train, label_names)
        train, test = chainer.datasets.split_dataset_random(
            dataset, int(len(dataset) * 0.8))
    else:
        train = SSDDataset(args.train, label_names)
        test  = SSDDataset(args.val, label_names)

    print("train: {}, test: {}".format(len(train), len(test)))

    pretrained_model = SSD300(pretrained_model=args.base_model)

    # copy from pretrained model
    model = SSD300(n_fg_class=len(dataset.label_names))
    model.extractor.copyparams(pretrained_model.extractor)
    model.multibox.loc.copyparams(pretrained_model.multibox.loc)

    model.use_preset("evaluate")

    train_chain = MultiboxTrainChain(model)

    if args.gpu >= 0:
        chainer.cuda.get_device(args.gpu).use()
        model.to_gpu()

    train = TransformDataset(
        train, Transform(model.coder, model.insize, model.mean))
    train_iter = chainer.iterators.MultiprocessIterator(
        train, args.batchsize)

    test_iter = chainer.iterators.SerialIterator(
        test, args.batchsize,
        repeat=False, shuffle=False)

    optimizer = chainer.optimizers.MomentumSGD(lr=args.lr)
    optimizer.setup(train_chain)

    for param in train_chain.params():
        if param.name == 'b':
            param.update_rule.add_hook(GradientScaling(2))
        else:
            param.update_rule.add_hook(WeightDecay(0.0005))

    updater = training.StandardUpdater(
        train_iter, optimizer, device=args.gpu)
    trainer = training.Trainer(
        updater, (args.iteration, "iteration"), args.out)

    val_interval = args.val_iter, "iteration"
    trainer.extend(
        DetectionVOCEvaluator(
            test_iter, model, use_07_metric=True,
            label_names=label_names),
        trigger=val_interval)

    log_interval = args.log_iter, "iteration"
    trainer.extend(extensions.LogReport(trigger=log_interval))
    trainer.extend(extensions.observe_lr(), trigger=log_interval)
    trainer.extend(extensions.PrintReport(
        ['epoch', 'iteration', 'lr',
         'main/loss', 'main/loss/loc', 'main/loss/conf',
         'validation/main/map']),
        trigger=log_interval)
    trainer.extend(extensions.ProgressBar(update_interval=10))

    trainer.extend(extensions.snapshot(), trigger=val_interval)
    trainer.extend(
        extensions.snapshot_object(model, 'model_iter_{.updater.iteration}'),
        trigger=(args.model_iter, 'iteration'))

    if args.resume:
        serializers.load_npz(args.resume, trainer)

    trainer.run()
