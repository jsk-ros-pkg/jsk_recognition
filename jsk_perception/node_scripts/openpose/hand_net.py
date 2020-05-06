#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The code is originated from
# https://github.com/DeNA/Chainer_Realtime_Multi-Person_Pose_Estimation/blob/master/models/HandNet.py

from __future__ import print_function

import os
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
import chainer.functions as F
import chainer.links as L


base_url = 'http://posefs1.perception.cs.cmu.edu/OpenPose/models/hand/'
models = {
    'auto': 'pose_iter_102000.chainermodel',
}


class HandNet(chainer.Chain):

    def __init__(self, pretrained_model='auto'):
        super(HandNet, self).__init__()
        with self.init_scope():
            self.conv1_1 = L.Convolution2D(
                in_channels=3, out_channels=64, ksize=3, stride=1, pad=1)
            self.conv1_2 = L.Convolution2D(
                in_channels=64, out_channels=64, ksize=3, stride=1, pad=1)
            self.conv2_1 = L.Convolution2D(
                in_channels=64, out_channels=128, ksize=3, stride=1, pad=1)
            self.conv2_2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=3, stride=1, pad=1)
            self.conv3_1 = L.Convolution2D(
                in_channels=128, out_channels=256, ksize=3, stride=1, pad=1)
            self.conv3_2 = L.Convolution2D(
                in_channels=256, out_channels=256, ksize=3, stride=1, pad=1)
            self.conv3_3 = L.Convolution2D(
                in_channels=256, out_channels=256, ksize=3, stride=1, pad=1)
            self.conv3_4 = L.Convolution2D(
                in_channels=256, out_channels=256, ksize=3, stride=1, pad=1)
            self.conv4_1 = L.Convolution2D(
                in_channels=256, out_channels=512, ksize=3, stride=1, pad=1)
            self.conv4_2 = L.Convolution2D(
                in_channels=512, out_channels=512, ksize=3, stride=1, pad=1)
            self.conv4_3 = L.Convolution2D(
                in_channels=512, out_channels=512, ksize=3, stride=1, pad=1)
            self.conv4_4 = L.Convolution2D(
                in_channels=512, out_channels=512, ksize=3, stride=1, pad=1)
            self.conv5_1 = L.Convolution2D(
                in_channels=512, out_channels=512, ksize=3, stride=1, pad=1)
            self.conv5_2 = L.Convolution2D(
                in_channels=512, out_channels=512, ksize=3, stride=1, pad=1)
            self.conv5_3_CPM = L.Convolution2D(
                in_channels=512, out_channels=128, ksize=3, stride=1, pad=1)
            # stage1
            self.conv6_1_CPM = L.Convolution2D(
                in_channels=128, out_channels=512, ksize=1, stride=1, pad=0)
            self.conv6_2_CPM = L.Convolution2D(
                in_channels=512, out_channels=22, ksize=1, stride=1, pad=0)
            # stage2
            self.Mconv1_stage2 = L.Convolution2D(
                in_channels=150, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv2_stage2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv3_stage2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv4_stage2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv5_stage2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv6_stage2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=1, stride=1, pad=0)
            self.Mconv7_stage2 = L.Convolution2D(
                in_channels=128, out_channels=22, ksize=1, stride=1, pad=0)
            # stage3
            self.Mconv1_stage3 = L.Convolution2D(
                in_channels=150, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv2_stage3 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv3_stage3 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv4_stage3 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv5_stage3 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv6_stage3 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=1, stride=1, pad=0)
            self.Mconv7_stage3 = L.Convolution2D(
                in_channels=128, out_channels=22, ksize=1, stride=1, pad=0)
            # stage4
            self.Mconv1_stage4 = L.Convolution2D(
                in_channels=150, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv2_stage4 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv3_stage4 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv4_stage4 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv5_stage4 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv6_stage4 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=1, stride=1, pad=0)
            self.Mconv7_stage4 = L.Convolution2D(
                in_channels=128, out_channels=22, ksize=1, stride=1, pad=0)
            # stage5
            self.Mconv1_stage5 = L.Convolution2D(
                in_channels=150, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv2_stage5 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv3_stage5 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv4_stage5 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv5_stage5 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv6_stage5 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=1, stride=1, pad=0)
            self.Mconv7_stage5 = L.Convolution2D(
                in_channels=128, out_channels=22, ksize=1, stride=1, pad=0)
            # stage6
            self.Mconv1_stage6 = L.Convolution2D(
                in_channels=150, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv2_stage6 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv3_stage6 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv4_stage6 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv5_stage6 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv6_stage6 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=1, stride=1, pad=0)
            self.Mconv7_stage6 = L.Convolution2D(
                in_channels=128, out_channels=22, ksize=1, stride=1, pad=0)

            if pretrained_model in models.keys():
                data_dir = chainer.dataset.get_dataset_directory('openpose/hand')
                model_path = os.path.join(data_dir, models[pretrained_model])
                try:
                    os.makedirs(os.path.dirname(model_path))
                except OSError:
                    pass
                chainer.dataset.cache_or_load_file(
                    model_path,
                    lambda f: _download_pretrained_model(pretrained_model, f),
                    lambda f: f)
                chainer.serializers.load_npz(model_path, self)
            elif pretrained_model is not None:
                if not os.path.exists(pretrained_model):
                    raise OSError('model does not exists: "%s"' % pretrained_model)
                chainer.serializers.load_npz(pretrained_model, self)

    def __call__(self, x):
        heatmaps = []

        h = F.relu(self.conv1_1(x))
        h = F.relu(self.conv1_2(h))
        h = F.max_pooling_2d(h, ksize=2, stride=2)
        h = F.relu(self.conv2_1(h))
        h = F.relu(self.conv2_2(h))
        h = F.max_pooling_2d(h, ksize=2, stride=2)
        h = F.relu(self.conv3_1(h))
        h = F.relu(self.conv3_2(h))
        h = F.relu(self.conv3_3(h))
        h = F.relu(self.conv3_4(h))
        h = F.max_pooling_2d(h, ksize=2, stride=2)
        h = F.relu(self.conv4_1(h))
        h = F.relu(self.conv4_2(h))
        h = F.relu(self.conv4_3(h))
        h = F.relu(self.conv4_4(h))
        h = F.relu(self.conv5_1(h))
        h = F.relu(self.conv5_2(h))
        h = F.relu(self.conv5_3_CPM(h))
        feature_map = h

        # stage1
        h = F.relu(self.conv6_1_CPM(h))
        h = self.conv6_2_CPM(h)
        heatmaps.append(h)

        # stage2
        h = F.concat((h, feature_map), axis=1) # channel concat
        h = F.relu(self.Mconv1_stage2(h))
        h = F.relu(self.Mconv2_stage2(h))
        h = F.relu(self.Mconv3_stage2(h))
        h = F.relu(self.Mconv4_stage2(h))
        h = F.relu(self.Mconv5_stage2(h))
        h = F.relu(self.Mconv6_stage2(h))
        h = self.Mconv7_stage2(h)
        heatmaps.append(h)

        # stage3
        h = F.concat((h, feature_map), axis=1) # channel concat
        h = F.relu(self.Mconv1_stage3(h))
        h = F.relu(self.Mconv2_stage3(h))
        h = F.relu(self.Mconv3_stage3(h))
        h = F.relu(self.Mconv4_stage3(h))
        h = F.relu(self.Mconv5_stage3(h))
        h = F.relu(self.Mconv6_stage3(h))
        h = self.Mconv7_stage3(h)
        heatmaps.append(h)

        # stage4
        h = F.concat((h, feature_map), axis=1) # channel concat
        h = F.relu(self.Mconv1_stage4(h))
        h = F.relu(self.Mconv2_stage4(h))
        h = F.relu(self.Mconv3_stage4(h))
        h = F.relu(self.Mconv4_stage4(h))
        h = F.relu(self.Mconv5_stage4(h))
        h = F.relu(self.Mconv6_stage4(h))
        h = self.Mconv7_stage4(h)
        heatmaps.append(h)

        # stage5
        h = F.concat((h, feature_map), axis=1) # channel concat
        h = F.relu(self.Mconv1_stage5(h))
        h = F.relu(self.Mconv2_stage5(h))
        h = F.relu(self.Mconv3_stage5(h))
        h = F.relu(self.Mconv4_stage5(h))
        h = F.relu(self.Mconv5_stage5(h))
        h = F.relu(self.Mconv6_stage5(h))
        h = self.Mconv7_stage5(h)
        heatmaps.append(h)

        # stage6
        h = F.concat((h, feature_map), axis=1) # channel concat
        h = F.relu(self.Mconv1_stage6(h))
        h = F.relu(self.Mconv2_stage6(h))
        h = F.relu(self.Mconv3_stage6(h))
        h = F.relu(self.Mconv4_stage6(h))
        h = F.relu(self.Mconv5_stage6(h))
        h = F.relu(self.Mconv6_stage6(h))
        h = self.Mconv7_stage6(h)
        heatmaps.append(h)

        return heatmaps


def _download_pretrained_model(model_type, dest_path):
    from chainer.links import caffe

    if os.path.exists(dest_path):
        raise OSError('destination already exists: %s' % dest_path)

    basename, ext = os.path.splitext(models[model_type])
    url = base_url + basename + '.caffemodel'
    caffe_model_path = chainer.dataset.cached_download(url)
    if not os.path.exists(caffe_model_path):
        raise OSError('caffe model does not exist: %s' % caffe_model_path)

    print('Converting to chainer model')
    caffe_model = caffe.CaffeFunction(caffe_model_path)
    chainer_model = HandNet(pretrained_model=None)
    for link in chainer_model.links():
        if not isinstance(link, chainer.Link) or not link.name:
            continue
        if eval('chainer_model.{0}.b.shape == caffe_model["{0}"].b.shape'.format(link.name)) and\
           eval('chainer_model.{0}.W.shape == caffe_model["{0}"].W.shape'.format(link.name)):
            exec('chainer_model.{0}.W.data = caffe_model["{0}"].W.data'.format(link.name))
            exec('chainer_model.{0}.b.data = caffe_model["{0}"].b.data'.format(link.name))
            print('Copied layer {0}'.format(link.name))
        else:
            print('Failed to copy layer {0}'.format(link.name))

    chainer.serializers.save_npz(dest_path, chainer_model)
    return True
