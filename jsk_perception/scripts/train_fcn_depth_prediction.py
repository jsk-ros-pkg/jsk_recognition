#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import datetime
import os
import os.path as osp

import chainer
from chainer import cuda
from chainer.datasets import TransformDataset
from chainer.training import extensions
import cv2
import fcn
from jsk_recognition_utils.chainermodels import FCN8sDepthPredictionConcatFirst
from jsk_recognition_utils.datasets import DepthPredictionDataset
import numpy as np
import rospkg


def colorize_depth(depth, min_value=None, max_value=None):
    min_value = np.nanmin(depth) if min_value is None else min_value
    max_value = np.nanmax(depth) if max_value is None else max_value

    gray_depth = depth.copy()
    nan_mask = np.isnan(gray_depth)
    gray_depth[nan_mask] = 0
    gray_depth = 255 * (gray_depth - min_value) / (max_value - min_value)
    gray_depth[gray_depth < 0] = 0
    gray_depth[gray_depth > 255] = 255
    gray_depth = gray_depth.astype(np.uint8)
    colorized = cv2.applyColorMap(gray_depth, cv2.COLORMAP_JET)
    colorized[nan_mask] = (0, 0, 0)

    return colorized


def transform(in_data):
    min_value = 0.5
    max_value = 5.0

    label_gt = in_data[0][2]
    depth_gt = in_data[0][3]

    image_rgb, depth, label_gt, depth_gt, _ = in_data

    # RGB -> BGR
    image_bgr = image_rgb[:, :, ::-1]
    image_bgr = image_rgb.astype(np.float32)
    mean_bgr = np.array([104.00698793, 116.66876762, 122.67891434])
    image_bgr -= mean_bgr
    # (H, W, 3) -> (3, H, W)
    image_bgr = image_bgr.transpose((2, 0, 1))

    # depth -> depth_bgr: (H, W) -> (H, W, 3) -> (3, H, W)
    depth_bgr = colorize_depth(
        depth, min_value=min_value, max_value=max_value)
    depth_bgr = depth_bgr.astype(np.float32)
    depth_bgr -= mean_bgr
    depth_bgr = depth_bgr.transpose((2, 0, 1))

    return image_bgr, depth_bgr, label_gt, depth_gt


def main():
    rospack = rospkg.RosPack()
    jsk_perception_datasets_path = osp.join(
        rospack.get_path('jsk_perception'), 'learning_datasets')

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-g', '--gpu', default=0, type=int, help='GPU id')
    parser.add_argument(
        '-d', '--dataset_dir',
        default=osp.join(
            jsk_perception_datasets_path, 'human_size_mirror_dataset'),
        type=str, help='Path to root directory of dataset')
    parser.add_argument(
        '-m', '--model', default='FCN8sDepthPredictionConcatFirst', type=str,
        help='Model class name')
    parser.add_argument(
        '-b', '--batch_size', default=1, type=int, help='Batch size')
    parser.add_argument(
        '-e', '--epoch', default=100, type=int, help='Training epoch')
    parser.add_argument(
        '-o', '--out', type=str, default=None, help='Output directory')
    args = parser.parse_args()

    gpu = args.gpu
    out = args.out

    # 0. config

    timestamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    if out is None:
        out = osp.join(rospkg.get_ros_home(), 'learning_logs', timestamp)

    max_iter_epoch = args.epoch, 'epoch'
    progress_bar_update_interval = 10  # iteration
    print_interval = 100, 'iteration'
    log_interval = 100, 'iteration'
    test_interval = 5, 'epoch'
    save_interval = 5, 'epoch'

    # 1. dataset

    dataset_train = DepthPredictionDataset(
        args.dataset_dir, split='train', aug=True)
    dataset_valid = DepthPredictionDataset(
        args.dataset_dir, split='test', aug=False)

    dataset_train_transformed = TransformDataset(dataset_train, transform)
    dataset_valid_transformed = TransformDataset(dataset_valid, transform)

    iter_train = chainer.iterators.MultiprocessIterator(
        dataset_train_transformed, batch_size=args.batch_size,
        shared_mem=10 ** 8)
    iter_valid = chainer.iterators.MultiprocessIterator(
        dataset_valid_transformed, batch_size=1, shared_mem=10 ** 8,
        repeat=False, shuffle=False)

    # 2. model

    vgg = fcn.models.VGG16()
    vgg_path = vgg.download()
    chainer.serializers.load_npz(vgg_path, vgg)

    n_class = len(dataset_train.class_names)
    assert n_class == 2

    if args.model == 'FCN8sDepthPredictionConcatFirst':
        model = FCN8sDepthPredictionConcatFirst(n_class=n_class, masking=True)
    else:
        print('Invalid model class.')
        exit(1)

    model.init_from_vgg16(vgg)

    if gpu >= 0:
        cuda.get_device_from_id(gpu).use()
        model.to_gpu()

    # 3. optimizer

    optimizer = chainer.optimizers.Adam(alpha=1.0e-5)
    optimizer.setup(model)
    optimizer.add_hook(chainer.optimizer.WeightDecay(rate=0.0005))

    updater = chainer.training.updater.StandardUpdater(
        iter_train, optimizer, device=gpu)

    trainer = chainer.training.Trainer(updater, max_iter_epoch, out=out)

    trainer.extend(extensions.ExponentialShift("alpha", 0.99997))

    if not osp.isdir(out):
        os.makedirs(out)

    with open(osp.join(out, 'dataset.txt'), 'w') as f:
        f.write(dataset_train.__class__.__name__)

    with open(osp.join(out, 'model.txt'), 'w') as f:
        f.write(model.__class__.__name__)

    with open(osp.join(out, 'batch_size.txt'), 'w') as f:
        f.write(str(args.batch_size))

    trainer.extend(
        extensions.snapshot_object(
            model,
            savefun=chainer.serializers.save_npz,
            filename='model_snapshot.npz'),
        trigger=chainer.training.triggers.MaxValueTrigger(
            'validation/main/depth_acc<0.10', save_interval))

    trainer.extend(
        extensions.dump_graph(
            root_name='main/loss',
            out_name='network_architecture.dot'))

    trainer.extend(
        extensions.LogReport(
            log_name='log.json',
            trigger=log_interval))

    trainer.extend(
        extensions.PlotReport([
            'main/loss',
            'validation/main/loss',
        ],
            file_name='loss_plot.png',
            x_key='epoch',
            trigger=(5, 'epoch')),
        trigger=(5, 'epoch'))

    trainer.extend(chainer.training.extensions.PrintReport([
        'iteration',
        'epoch',
        'elapsed_time',
        'lr',
        'main/loss',
        'main/seg_loss',
        'main/reg_loss',
        'main/miou',
        'main/depth_acc<0.03',
        'main/depth_acc<0.10',
        'main/depth_acc<0.30',
        'validation/main/miou',
        'validation/main/depth_acc<0.03',
        'validation/main/depth_acc<0.10',
        'validation/main/depth_acc<0.30',
    ]), trigger=print_interval)

    trainer.extend(
        extensions.observe_lr(),
        trigger=log_interval)
    trainer.extend(
        extensions.ProgressBar(update_interval=progress_bar_update_interval))
    trainer.extend(
        extensions.Evaluator(iter_valid, model, device=gpu),
        trigger=test_interval)

    trainer.run()


if __name__ == '__main__':
    main()
