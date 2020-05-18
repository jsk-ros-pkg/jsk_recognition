#!/usr/bin/env python

from __future__ import print_function

import argparse
import copy
import datetime
import numpy as np
import os.path as osp

import itertools, pkg_resources, sys
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
from chainer.datasets import TransformDataset
from chainer.optimizer_hooks import WeightDecay
from chainer import training
from chainer.training import extensions
from chainer.training import triggers

from chainercv.extensions import DetectionVOCEvaluator
from chainercv.links.model.ssd import GradientScaling
from chainercv.links.model.ssd import multibox_loss
from chainercv.links import SSD300
from chainercv.links import SSD512
from chainercv import transforms

from chainercv.links.model.ssd import random_crop_with_bbox_constraints
from chainercv.links.model.ssd import random_distort
from chainercv.links.model.ssd import resize_with_random_interpolation

from jsk_recognition_utils.datasets import DetectionDataset
from jsk_recognition_utils.datasets import BboxDetectionDataset
import rospkg

# https://docs.chainer.org/en/stable/tips.html#my-training-process-gets-stuck-when-using-multiprocessiterator
import cv2
cv2.setNumThreads(0)
chainer.config.cv_resize_backend = 'cv2'


class MultiboxTrainChain(chainer.Chain):

    def __init__(self, model, alpha=1, k=3):
        super(MultiboxTrainChain, self).__init__()
        with self.init_scope():
            self.model = model
        self.alpha = alpha
        self.k = k

    def forward(self, imgs, gt_mb_locs, gt_mb_labels):
        mb_locs, mb_confs = self.model(imgs)
        loc_loss, conf_loss = multibox_loss(
            mb_locs, mb_confs, gt_mb_locs, gt_mb_labels, self.k)
        loss = loc_loss * self.alpha + conf_loss

        chainer.reporter.report(
            {'loss': loss, 'loss/loc': loc_loss, 'loss/conf': conf_loss},
            self)

        return loss


class Transform(object):

    def __init__(self, coder, size, mean):
        # to send cpu, make a copy
        self.coder = copy.copy(coder)
        self.coder.to_cpu()

        self.size = size
        self.mean = mean

    def __call__(self, in_data):
        # There are five data augmentation steps
        # 1. Color augmentation
        # 2. Random expansion
        # 3. Random cropping
        # 4. Resizing with random interpolation
        # 5. Random horizontal flipping

        img, bbox, label = in_data

        # 1. Color augmentation
        img = random_distort(img)

        # 2. Random expansion
        if np.random.randint(2):
            img, param = transforms.random_expand(
                img, fill=self.mean, return_param=True)
            bbox = transforms.translate_bbox(
                bbox, y_offset=param['y_offset'], x_offset=param['x_offset'])

        # 3. Random cropping
        img, param = random_crop_with_bbox_constraints(
            img, bbox, return_param=True)
        bbox, param = transforms.crop_bbox(
            bbox, y_slice=param['y_slice'], x_slice=param['x_slice'],
            allow_outside_center=False, return_param=True)
        label = label[param['index']]

        # 4. Resizing with random interpolatation
        _, H, W = img.shape
        img = resize_with_random_interpolation(img, (self.size, self.size))
        bbox = transforms.resize_bbox(bbox, (H, W), (self.size, self.size))

        # 5. Random horizontal flipping
        img, params = transforms.random_flip(
            img, x_random=True, return_param=True)
        bbox = transforms.flip_bbox(
            bbox, (self.size, self.size), x_flip=params['x_flip'])

        # Preparation for SSD network
        img -= self.mean
        mb_loc, mb_label = self.coder.encode(bbox, label)

        return img, mb_loc, mb_label


def main():
    rospack = rospkg.RosPack()
    jsk_perception_datasets_path = osp.join(
        rospack.get_path('jsk_perception'), 'learning_datasets')

    parser = argparse.ArgumentParser()
    # Dataset directory
    parser.add_argument('--train-dataset-dir', type=str,
                        default=osp.join(jsk_perception_datasets_path,
                                         'kitchen_dataset', 'train'))
    parser.add_argument('--val-dataset-dir', type=str,
                        default=osp.join(jsk_perception_datasets_path,
                                         'kitchen_dataset', 'test'))
    parser.add_argument('--dataset-type', type=str,
                        default='instance')
    parser.add_argument(
        '--model-name', choices=('ssd300', 'ssd512'), default='ssd512')
    parser.add_argument('--gpu', type=int, default=0)
    parser.add_argument('--batch-size', type=int, default=8)
    parser.add_argument('--max-epoch', type=int, default=100)
    parser.add_argument('--out-dir', type=str, default=None)
    args = parser.parse_args()

    if (args.dataset_type == 'instance'):
        train_dataset = DetectionDataset(args.train_dataset_dir)
    elif (args.dataset_type == 'bbox'):
        train_dataset = BboxDetectionDataset(args.train_dataset_dir)
    else:
        print('unsuppported dataset type')
        return

    fg_label_names = train_dataset.fg_class_names

    if args.model_name == 'ssd300':
        model = SSD300(
            n_fg_class=len(fg_label_names),
            pretrained_model='imagenet')
    elif args.model_name == 'ssd512':
        model = SSD512(
            n_fg_class=len(fg_label_names),
            pretrained_model='imagenet')

    model.use_preset('evaluate')
    train_chain = MultiboxTrainChain(model)
    if args.gpu >= 0:
        chainer.cuda.get_device_from_id(args.gpu).use()
        model.to_gpu()

    train = TransformDataset(
        train_dataset,
        Transform(model.coder, model.insize, model.mean))
    train_iter = chainer.iterators.MultiprocessIterator(train, args.batch_size)

    if (args.dataset_type == 'instance'):
        test_dataset = DetectionDataset(args.val_dataset_dir)
    elif (args.dataset_type == 'bbox'):
        test_dataset = BboxDetectionDataset(args.val_dataset_dir)

    test_iter = chainer.iterators.SerialIterator(
        test_dataset, args.batch_size, repeat=False, shuffle=False)

    # initial lr is set to 1e-3 by ExponentialShift
    optimizer = chainer.optimizers.MomentumSGD()
    optimizer.setup(train_chain)
    for param in train_chain.params():
        if param.name == 'b':
            param.update_rule.add_hook(GradientScaling(2))
        else:
            param.update_rule.add_hook(WeightDecay(0.0005))

    updater = training.updaters.StandardUpdater(
        train_iter, optimizer, device=args.gpu)

    now = datetime.datetime.now()
    timestamp = now.strftime('%Y%m%d-%H%M%S')
    if args.out_dir is None:
        out_dir = osp.join(
            rospkg.get_ros_home(), 'learning_logs', timestamp)
    else:
        out_dir = args.out_dir

    step_epoch = [args.max_epoch * 2 // 3, args.max_epoch * 5 // 6]
    trainer = training.Trainer(
        updater, (args.max_epoch, 'epoch'), out_dir)
    trainer.extend(
        extensions.ExponentialShift('lr', 0.1, init=1e-3),
        trigger=triggers.ManualScheduleTrigger(step_epoch, 'epoch'))

    trainer.extend(
        DetectionVOCEvaluator(
            test_iter, model, use_07_metric=True,
            label_names=fg_label_names),
        trigger=triggers.ManualScheduleTrigger(
            step_epoch + [args.max_epoch], 'epoch'))

    log_interval = 10, 'iteration'
    trainer.extend(
        extensions.LogReport(log_name='log.json', trigger=log_interval))
    trainer.extend(extensions.observe_lr(), trigger=log_interval)
    trainer.extend(extensions.PrintReport(
        ['epoch', 'iteration', 'lr',
         'main/loss', 'main/loss/loc', 'main/loss/conf',
         'validation/main/map']),
        trigger=log_interval)
    trainer.extend(extensions.ProgressBar(update_interval=10))

    trainer.extend(
        extensions.snapshot_object(
            model, 'model_snapshot.npz'),
        trigger=(args.max_epoch, 'epoch'))

    trainer.run()


if __name__ == '__main__':
    main()
