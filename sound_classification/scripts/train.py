#!/usr/bin/env python

# mainly copied from chainer/train_imagenet.py
# https://github.com/chainer/chainer/blob/master/examples/imagenet/train_imagenet.py

import argparse

import chainer
from chainer import dataset
from chainer import training
from chainer.links import VGG16Layers
import chainer.backends.cuda
from chainer.serializers import npz
from chainer.training import extensions

from sound_classification.nin.nin import NIN
from sound_classification.vgg16.vgg16_batch_normalization import VGG16BatchNormalization

import matplotlib
import numpy as np
from os import makedirs
import os.path as osp
from PIL import Image as Image_
import rospkg

matplotlib.use('Agg')  # necessary not to raise Tcl_AsyncDelete Error


class PreprocessedDataset(chainer.dataset.DatasetMixin):

    def __init__(self, path=None, random=True):
        rospack = rospkg.RosPack()
        # Root directory path of train data
        self.root = osp.join(rospack.get_path(
            'sound_classification'), 'train_data')
        if path is not None:
            self.base = chainer.datasets.LabeledImageDataset(
                path, osp.join(self.root, 'dataset'))
        self.random = random
        # how many classes to be classified
        self.n_class = 0
        self.target_classes = []
        with open(osp.join(self.root, 'n_class.txt'), mode='r') as f:
            for row in f:
                self.n_class += 1
                self.target_classes.append(row)
        # Load mean image of dataset
        mean_img_path = osp.join(rospack.get_path('sound_classification'),
                                 'train_data', 'dataset', 'mean_of_dataset.png')
        mean = np.array(Image_.open(mean_img_path), np.float32).transpose(
            (2, 0, 1))  # (height, width, channel) -> (channel ,height, width), rgb
        self.mean = mean.astype(chainer.get_dtype())

    def __len__(self):
        return len(self.base)

    def get_example(self, i):
        image, label = self.base[i]  # (channel ,height, width), rgb
        image = self.process_image(image)
        return image, label

    def process_image(self, image):
        ret = image - self.mean  # Subtract mean image, (channel ,height, width), rgb
        ret *= (1.0 / 255.0)  # Scale to [0, 1.0]
        return ret


def load_model(model_name, n_class):
    archs = {
        'nin': NIN,
        'vgg16': VGG16BatchNormalization
    }
    model = archs[model_name](n_class=n_class)
    if model_name == 'nin':
        pass
    elif model_name == 'vgg16':
        rospack = rospkg.RosPack()
        model_path = osp.join(rospack.get_path('sound_classification'), 'scripts',
                              'vgg16', 'VGG_ILSVRC_16_layers.npz')
        if not osp.exists(model_path):
            from chainer.dataset import download
            from chainer.links.caffe.caffe_function import CaffeFunction
            path_caffemodel = download.cached_download('http://www.robots.ox.ac.uk/%7Evgg/software/very_deep/caffe/VGG_ILSVRC_19_layers.caffemodel')
            caffemodel = CaffeFunction(path_caffemodel)
            npz.save_npz(model_path, caffemodel, compression=False)

        vgg16 = VGG16Layers(pretrained_model=model_path)  # original VGG16 model
        print('Load model from {}'.format(model_path))
        for l in model.children():
            if l.name.startswith('conv'):
                # l.disable_update()  # Comment-in for transfer learning, comment-out for fine tuning
                l1 = getattr(vgg16, l.name)
                l2 = getattr(model, l.name)
                assert l1.W.shape == l2.W.shape
                assert l1.b.shape == l2.b.shape
                l2.W.data[...] = l1.W.data[...]
                l2.b.data[...] = l1.b.data[...]
            elif l.name in ['fc6', 'fc7']:
                l1 = getattr(vgg16, l.name)
                l2 = getattr(model, l.name)
                assert l1.W.size == l2.W.size
                assert l1.b.size == l2.b.size
                l2.W.data[...] = l1.W.data.reshape(l2.W.shape)[...]
                l2.b.data[...] = l1.b.data.reshape(l2.b.shape)[...]
    else:
        print('Model type {} is invalid.'.format(model_name))
        exit()

    return model


def main():
    rospack = rospkg.RosPack()

    parser = argparse.ArgumentParser(
        description='Learning convnet from ILSVRC2012 dataset')
    parser.add_argument('--epoch', '-e', type=int, default=100,
                        help='Number of epochs to train')
    parser.add_argument('--gpu', '-g', type=int, default=0,
                        help='GPU ID (negative value indicates CPU)')
    parser.add_argument('-m', '--model', type=str,
                        choices=['nin', 'vgg16'], default='nin',
                        help='Neural network model to use dataset')
    # Ignore arguments sent by roslaunch.
    parser.add_argument('__name:', help=argparse.SUPPRESS, nargs='?')
    parser.add_argument('__log:', help=argparse.SUPPRESS, nargs='?')

    args = parser.parse_args()

    # Configs for train with chainer
    if args.gpu >= 0:
        device = chainer.cuda.get_device_from_id(args.gpu)  # for python2
    else:
        device = None
    batchsize = 32
    # Path to training image-label list file
    train_labels = osp.join(rospack.get_path('sound_classification'),
                            'train_data', 'dataset', 'train_images.txt')
    # Path to validation image-label list file
    val_labels = osp.join(rospack.get_path('sound_classification'),
                          'train_data', 'dataset', 'test_images.txt')

    # Initialize the model to train
    print('Device: {}'.format(device))
    print('Model: {}'.format(args.model))
    print('Dtype: {}'.format(chainer.config.dtype))
    print('Minibatch-size: {}'.format(batchsize))
    print('epoch: {}'.format(args.epoch))
    print('')

    # Load the dataset files
    train = PreprocessedDataset(train_labels)
    val = PreprocessedDataset(val_labels, False)

    model = load_model(args.model, train.n_class)
    if device is not None:
        if hasattr(model, 'to_device'):
            model.to_device(device)
            device.use()
        else:
            model.to_gpu(device)

    # These iterators load the images with subprocesses running in parallel
    # to the training/validation.
    train_iter = chainer.iterators.MultiprocessIterator(
        train, batchsize)
    val_iter = chainer.iterators.MultiprocessIterator(
        val, batchsize, repeat=False)
    converter = dataset.concat_examples

    # Set up an optimizer
    optimizer = chainer.optimizers.MomentumSGD(lr=0.01, momentum=0.9)
    optimizer.setup(model)

    # Set up a trainer
    # Output directory of train result
    out = osp.join(rospack.get_path('sound_classification'),
                   'train_data', 'result', args.model)
    if not osp.exists(out):
        makedirs(out)
    updater = training.updaters.StandardUpdater(
        train_iter, optimizer, converter=converter, device=device)
    trainer = training.Trainer(updater, (args.epoch, 'epoch'), out)

    val_interval = 10, 'iteration'
    log_interval = 10, 'iteration'

    trainer.extend(extensions.Evaluator(val_iter, model, converter=converter,
                                        device=device), trigger=val_interval)
    trainer.extend(extensions.snapshot_object(
        target=model, filename='model_best.npz'),
        trigger=chainer.training.triggers.MinValueTrigger(
            key='validation/main/loss',
            trigger=val_interval))
    # Be careful to pass the interval directly to LogReport
    # (it determines when to emit log rather than when to read observations)
    trainer.extend(extensions.LogReport(trigger=log_interval))
    trainer.extend(extensions.observe_lr(), trigger=log_interval)
    trainer.extend(extensions.PrintReport([
        'epoch', 'iteration', 'main/loss', 'validation/main/loss',
        'main/accuracy', 'validation/main/accuracy', 'lr'
    ]), trigger=log_interval)
    trainer.extend(extensions.PlotReport(['main/loss', 'validation/main/loss'], x_key='iteration', file_name='loss.png'))
    trainer.extend(extensions.PlotReport(['main/accuracy', 'validation/main/accuracy'], x_key='iteration', file_name='accuracy.png'))
    trainer.extend(extensions.ProgressBar(update_interval=10))

    trainer.run()


if __name__ == '__main__':
    main()
