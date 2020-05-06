#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

import argparse
import datetime
import functools
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
from chainer.training import extensions
import chainer_mask_rcnn as cmr
import fcn

from jsk_recognition_utils.datasets import InstanceSegmentationDataset
import rospkg


class TrainMaskRCNN(object):

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
            '--model_name', type=str, default='resnet50',
            choices=['vgg16', 'resnet50', 'resnet101'])

        # Training parameters
        parser.add_argument('--gpu', type=int, default=0)
        parser.add_argument('--batch_size', type=int, default=1)
        parser.add_argument('--max_epoch', type=int, default=100)
        parser.add_argument('--lr', type=float, default=0.00125)
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
        self.load_model()
        self.setup_optimizer()
        self.setup_iterator()
        self.setup_trainer()
        self.trainer.run()

    def load_dataset(self):
        self.train_dataset = InstanceSegmentationDataset(
            self.train_dataset_dir)
        self.val_dataset = InstanceSegmentationDataset(self.val_dataset_dir)

    def load_model(self):
        n_fg_class = len(self.train_dataset.fg_class_names)

        pooling_func = cmr.functions.roi_align_2d
        anchor_scales = (4, 8, 16, 32)
        roi_size = 14
        min_size = 600
        max_size = 1000
        mask_initialW = chainer.initializers.Normal(0.01)

        if self.model_name == 'vgg16':
            self.mask_rcnn = cmr.models.MaskRCNNVGG16(
                n_fg_class=n_fg_class,
                pretrained_model='imagenet',
                pooling_func=pooling_func,
                anchor_scales=anchor_scales,
                roi_size=roi_size,
                min_size=min_size,
                max_size=max_size,
                mask_initialW=mask_initialW,
            )
        elif self.model_name in ['resnet50', 'resnet101']:
            n_layers = int(self.model_name.lstrip('resnet'))
            self.mask_rcnn = cmr.models.MaskRCNNResNet(
                n_layers=n_layers,
                n_fg_class=n_fg_class,
                pooling_func=pooling_func,
                anchor_scales=anchor_scales,
                roi_size=roi_size,
                min_size=min_size,
                max_size=max_size,
                mask_initialW=mask_initialW,
            )
        else:
            raise ValueError(
                'Unsupported model_name: {}'.format(self.model_name))
        self.model = cmr.models.MaskRCNNTrainChain(self.mask_rcnn)

        if self.gpu >= 0:
            cuda.get_device_from_id(self.gpu).use()
            self.model.to_gpu()

    def setup_optimizer(self):
        self.optimizer = chainer.optimizers.MomentumSGD(
            lr=self.lr, momentum=0.9)
        self.optimizer.setup(self.model)
        self.optimizer.add_hook(
            chainer.optimizer.WeightDecay(rate=self.weight_decay))

        if self.model_name in ['resnet50', 'resnet101']:
            # ResNetExtractor.freeze_at is not enough to freeze params
            # since WeightDecay updates the param little by little.
            self.mask_rcnn.extractor.conv1.disable_update()
            self.mask_rcnn.extractor.bn1.disable_update()
            self.mask_rcnn.extractor.res2.disable_update()
            for link in self.mask_rcnn.links():
                if isinstance(link, cmr.links.AffineChannel2D):
                    link.disable_update()

    def setup_iterator(self):
        train_dataset_transformed = TransformDataset(
            self.train_dataset, cmr.datasets.MaskRCNNTransform(self.mask_rcnn))
        val_dataset_transformed = TransformDataset(
            self.val_dataset,
            cmr.datasets.MaskRCNNTransform(self.mask_rcnn, train=False))
        # FIXME: MultiProcessIterator sometimes hangs
        self.train_iterator = chainer.iterators.SerialIterator(
            train_dataset_transformed, batch_size=self.batch_size)
        self.val_iterator = chainer.iterators.SerialIterator(
            val_dataset_transformed, batch_size=self.batch_size,
            repeat=False, shuffle=False)

    def setup_trainer(self):
        converter = functools.partial(
            cmr.datasets.concat_examples,
            padding=0,
            # img, bboxes, labels, masks, scales
            indices_concat=[0, 2, 3, 4],  # img, _, labels, masks, scales
            indices_to_device=[0, 1],  # img, bbox
        )
        self.updater = chainer.training.updater.StandardUpdater(
            self.train_iterator, self.optimizer, device=self.gpu,
            converter=converter)
        self.trainer = chainer.training.Trainer(
            self.updater, (self.max_epoch, 'epoch'), out=self.out_dir)

        step_size = [
            (120e3 / 180e3) * self.max_epoch,
            (160e3 / 180e3) * self.max_epoch,
        ]
        self.trainer.extend(
            extensions.ExponentialShift('lr', 0.1),
            trigger=chainer.training.triggers.ManualScheduleTrigger(
                step_size, 'epoch'))

        evaluator = cmr.extensions.InstanceSegmentationVOCEvaluator(
            self.val_iterator, self.model.mask_rcnn, device=self.gpu,
            use_07_metric=True, label_names=self.train_dataset.fg_class_names)
        self.trainer.extend(
            evaluator, trigger=(self.eval_interval, self.eval_interval_type))

        # Save snapshot
        self.trainer.extend(
            extensions.snapshot_object(
                self.model.mask_rcnn, 'snapshot_model.npz'),
            trigger=chainer.training.triggers.MaxValueTrigger(
                'validation/main/map',
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
                'main/roi_loc_loss',
                'main/roi_cls_loss',
                'main/roi_mask_loss',
                'main/rpn_loc_loss',
                'main/rpn_cls_loss',
                'validation/main/map',
            ]), trigger=(self.print_interval, self.print_interval_type))

        # Plot
        self.trainer.extend(
            extensions.PlotReport([
                'main/loss',
                'main/roi_loc_loss',
                'main/roi_cls_loss',
                'main/roi_mask_loss',
                'main/rpn_loc_loss',
                'main/rpn_cls_loss',
            ],
                file_name='loss_plot.png',
                x_key=self.plot_interval_type,
                trigger=(self.plot_interval, self.plot_interval_type)),
            trigger=(self.plot_interval, self.plot_interval_type))
        self.trainer.extend(
            extensions.PlotReport(
                ['validation/main/map'],
                file_name='accuracy_plot.png',
                x_key=self.plot_interval_type,
                trigger=(self.plot_interval, self.plot_interval_type)),
            trigger=(self.eval_interval, self.eval_interval_type))

        # Dump params
        params = dict()
        params['model_name'] = self.model_name
        params['train_dataset_dir'] = self.train_dataset_dir
        params['val_dataset_dir'] = self.val_dataset_dir
        params['fg_class_names'] = self.train_dataset.fg_class_names
        params['timestamp'] = self.timestamp_iso
        params['out_dir'] = self.out_dir
        params['gpu'] = self.gpu
        params['batch_size'] = self.batch_size
        params['max_epoch'] = self.max_epoch
        params['lr'] = self.lr
        params['weight_decay'] = self.weight_decay
        self.trainer.extend(
            fcn.extensions.ParamsReport(params, file_name='params.yaml'))

        # Dump param for mask_rcnn_instance_segmentation.py
        target_names = dict()
        target_names['fg_class_names'] = self.train_dataset.fg_class_names
        self.trainer.extend(
            fcn.extensions.ParamsReport(
                target_names, file_name='fg_class_names.yaml'))


if __name__ == '__main__':
    app = TrainMaskRCNN()
