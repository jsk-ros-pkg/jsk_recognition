#!/usr/bin/env python

from __future__ import print_function

import argparse
import datetime
import os
import os.path as osp

os.environ['MPLBACKEND'] = 'Agg'  # NOQA

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
from chainer import cuda
from chainer.datasets import TransformDataset
import chainer.serializers as S
from chainer.training import extensions
import fcn
import numpy as np

from jsk_recognition_utils.datasets import SemanticSegmentationDataset
import rospkg


class TrainFCN(object):

    def __init__(self):
        rospack = rospkg.RosPack()
        jsk_perception_datasets_path = osp.join(
            rospack.get_path('jsk_perception'), 'learning_datasets')

        parser = argparse.ArgumentParser()

        # Dataset directory
        parser.add_argument('--train_dataset_dir', type=str,
                            default=osp.join(jsk_perception_datasets_path,
                                             'kitchen_dataset', 'train'))
        parser.add_argument('--val_dataset_dir', type=str,
                            default=osp.join(jsk_perception_datasets_path,
                                             'kitchen_dataset', 'test'))

        # Model
        parser.add_argument(
            '--model_name', type=str, default='fcn32s',
            choices=['fcn32s', 'fcn16s', 'fcn8s', 'fcn8s_at_once'])

        # Training parameters
        parser.add_argument('--gpu', type=int, default=0)
        parser.add_argument('--batch_size', type=int, default=1)
        parser.add_argument('--max_epoch', type=int, default=100)
        parser.add_argument('--lr', type=float, default=1e-10)
        parser.add_argument('--weight_decay', type=float, default=0.0001)
        parser.add_argument('--out_dir', type=str, default=None)
        parser.add_argument('--progressbar_update_interval', type=float,
                            default=10)
        parser.add_argument('--print_interval', type=float, default=100)
        parser.add_argument('--print_interval_type', type=str,
                            default='iteration',
                            choices=['epoch', 'iteration'])
        parser.add_argument('--log_interval', type=float, default=10)
        parser.add_argument('--log_interval_type', type=str,
                            default='iteration',
                            choices=['epoch', 'iteration'])
        parser.add_argument('--plot_interval', type=float, default=5)
        parser.add_argument('--plot_interval_type', type=str,
                            default='epoch',
                            choices=['epoch', 'iteration'])
        parser.add_argument('--eval_interval', type=float, default=10)
        parser.add_argument('--eval_interval_type', type=str,
                            default='epoch',
                            choices=['epoch', 'iteration'])
        parser.add_argument('--save_interval', type=float, default=10)
        parser.add_argument('--save_interval_type', type=str,
                            default='epoch',
                            choices=['epoch', 'iteration'])

        args = parser.parse_args()

        self.train_dataset_dir = args.train_dataset_dir
        self.val_dataset_dir = args.val_dataset_dir
        self.model_name = args.model_name
        self.gpu = args.gpu
        self.batch_size = args.batch_size
        self.max_epoch = args.max_epoch
        self.lr = args.lr
        self.weight_decay = args.weight_decay
        self.out_dir = args.out_dir
        self.progressbar_update_interval = args.progressbar_update_interval
        self.print_interval = args.print_interval
        self.print_interval_type = args.print_interval_type
        self.log_interval = args.log_interval
        self.log_interval_type = args.log_interval_type
        self.plot_interval = args.plot_interval
        self.plot_interval_type = args.plot_interval_type
        self.eval_interval = args.eval_interval
        self.eval_interval_type = args.eval_interval_type
        self.save_interval = args.save_interval
        self.save_interval_type = args.save_interval_type

        now = datetime.datetime.now()
        self.timestamp_iso = now.isoformat()
        timestamp = now.strftime('%Y%m%d-%H%M%S')
        if self.out_dir is None:
            self.out_dir = osp.join(
                rospkg.get_ros_home(), 'learning_logs', timestamp)

        # Main process
        self.load_dataset()
        self.setup_iterator()
        self.load_model()
        self.setup_optimizer()
        self.setup_trainer()
        self.trainer.run()

    def load_dataset(self):
        self.train_dataset = SemanticSegmentationDataset(
            self.train_dataset_dir)
        self.val_dataset = SemanticSegmentationDataset(self.val_dataset_dir)

    def transform_dataset(self, in_data):
        rgb_img, lbl = in_data
        # RGB -> BGR
        mean_bgr = np.array([104.00698793, 116.66876762, 122.67891434])
        bgr_img = rgb_img[:, :, ::-1]
        bgr_img = bgr_img.astype(np.float32)
        bgr_img -= mean_bgr
        # H, W, C -> C, H, W
        bgr_img = bgr_img.transpose((2, 0, 1))

        return bgr_img, lbl

    def setup_iterator(self):
        train_dataset_transformed = TransformDataset(
            self.train_dataset, self.transform_dataset)
        val_dataset_transformed = TransformDataset(
            self.val_dataset, self.transform_dataset)
        self.train_iterator = chainer.iterators.MultiprocessIterator(
            train_dataset_transformed, batch_size=self.batch_size,
            shared_mem=10 ** 7)
        self.val_iterator = chainer.iterators.MultiprocessIterator(
            val_dataset_transformed, batch_size=self.batch_size,
            shared_mem=10 ** 7, repeat=False, shuffle=False)

    def load_model(self):
        n_class = len(self.train_dataset.class_names)
        if self.model_name == 'fcn32s':
            self.model = fcn.models.FCN32s(n_class=n_class)
            vgg = fcn.models.VGG16()
            vgg_path = vgg.download()
            S.load_npz(vgg_path, vgg)
            self.model.init_from_vgg16(vgg)
        elif self.model_name == 'fcn16s':
            self.model = fcn.models.FCN16s(n_class=n_class)
            fcn32s = fcn.models.FCN32s()
            fcn32s_path = fcn32s.download()
            S.load_npz(fcn32s_path, fcn32s)
            self.model.init_from_fcn32s(fcn32s_path, fcn32s)
        elif self.model_name == 'fcn8s':
            self.model = fcn.models.FCN8s(n_class=n_class)
            fcn16s = fcn.models.FCN16s()
            fcn16s_path = fcn16s.download()
            S.load_npz(fcn16s_path, fcn16s)
            self.model.init_from_fcn16s(fcn16s_path, fcn16s)
        elif self.model_name == 'fcn8s_at_once':
            self.model = fcn.models.FCN8sAtOnce(n_class=n_class)
            vgg = fcn.models.VGG16()
            vgg_path = vgg.download()
            S.load_npz(vgg_path, vgg)
            self.model.init_from_vgg16(vgg)
        else:
            raise ValueError(
                'Unsupported model_name: {}'.format(self.model_name))

        if self.gpu >= 0:
            cuda.get_device_from_id(self.gpu).use()
            self.model.to_gpu()

    def setup_optimizer(self):
        self.optimizer = chainer.optimizers.MomentumSGD(
            lr=self.lr, momentum=0.9)
        self.optimizer.setup(self.model)
        self.optimizer.add_hook(
            chainer.optimizer.WeightDecay(rate=self.weight_decay))

    def setup_trainer(self):
        self.updater = chainer.training.updater.StandardUpdater(
            self.train_iterator, self.optimizer, device=self.gpu)
        self.trainer = chainer.training.Trainer(
            self.updater, (self.max_epoch, 'epoch'), out=self.out_dir)

        self.trainer.extend(
            extensions.Evaluator(
                self.val_iterator, self.model, device=self.gpu),
            trigger=(self.eval_interval, self.eval_interval_type))

        # Save snapshot
        self.trainer.extend(
            extensions.snapshot_object(
                self.model,
                savefun=S.save_npz,
                filename='model_snapshot.npz'),
            trigger=chainer.training.triggers.MinValueTrigger(
                'validation/main/loss',
                (self.save_interval, self.save_interval_type)))

        # Dump network architecture
        self.trainer.extend(
            extensions.dump_graph(
                root_name='main/loss',
                out_name='network_architecture.dot'))

        # Logging
        self.trainer.extend(
            extensions.ProgressBar(
                update_interval=self.progressbar_update_interval))
        self.trainer.extend(
            extensions.observe_lr(),
            trigger=(self.log_interval, self.log_interval_type))
        self.trainer.extend(
            extensions.LogReport(
                log_name='log.json',
                trigger=(self.log_interval, self.log_interval_type)))
        self.trainer.extend(
            extensions.PrintReport([
                'iteration',
                'epoch',
                'elapsed_time',
                'lr',
                'main/loss',
                'validation/main/loss',
            ]), trigger=(self.print_interval, self.print_interval_type))

        # Plot
        self.trainer.extend(
            extensions.PlotReport([
                'main/loss',
                'validation/main/loss',
            ],
                file_name='loss_plot.png',
                x_key=self.plot_interval_type,
                trigger=(self.plot_interval, self.plot_interval_type)),
            trigger=(self.plot_interval, self.plot_interval_type))

        # Dump params
        params = dict()
        params['model_name'] = self.model_name
        params['train_dataset_dir'] = self.train_dataset_dir
        params['val_dataset_dir'] = self.val_dataset_dir
        params['class_names'] = self.train_dataset.class_names
        params['timestamp'] = self.timestamp_iso
        params['out_dir'] = self.out_dir
        params['gpu'] = self.gpu
        params['batch_size'] = self.batch_size
        params['max_epoch'] = self.max_epoch
        params['lr'] = self.lr
        params['weight_decay'] = self.weight_decay
        self.trainer.extend(
            fcn.extensions.ParamsReport(params, file_name='params.yaml'))

        # Dump param for fcn_object_segmentation.py
        model_name = dict()
        model_name['model_name'] = self.model_name
        self.trainer.extend(
            fcn.extensions.ParamsReport(
                model_name, file_name='model_name.yaml'))
        target_names = dict()
        target_names['target_names'] = self.train_dataset.class_names
        self.trainer.extend(
            fcn.extensions.ParamsReport(
                target_names, file_name='target_names.yaml'))


if __name__ == '__main__':
    app = TrainFCN()
