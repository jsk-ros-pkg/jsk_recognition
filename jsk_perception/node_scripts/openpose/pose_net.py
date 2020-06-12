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

base_url = 'http://posefs1.perception.cs.cmu.edu/OpenPose/models/pose/'
models = {
    'auto': 'coco/pose_iter_440000.chainermodel',
    'coco': 'coco/pose_iter_440000.chainermodel',
    'mpi': 'mpi/pose_iter_160000.chainermodel',
}


class PoseNet(chainer.Chain):

    def __init__(self, pretrained_model='auto'):
        super(PoseNet, self).__init__()
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
            self.conv4_3_CPM = L.Convolution2D(
                in_channels=512, out_channels=256, ksize=3, stride=1, pad=1)
            self.conv4_4_CPM = L.Convolution2D(
                in_channels=256, out_channels=128, ksize=3, stride=1, pad=1)

            # stage1
            self.conv5_1_CPM_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=3, stride=1, pad=1)
            self.conv5_2_CPM_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=3, stride=1, pad=1)
            self.conv5_3_CPM_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=3, stride=1, pad=1)
            self.conv5_4_CPM_L1 = L.Convolution2D(
                in_channels=128, out_channels=512, ksize=1, stride=1, pad=0)
            self.conv5_5_CPM_L1 = L.Convolution2D(
                in_channels=512, out_channels=38, ksize=1, stride=1, pad=0)
            self.conv5_1_CPM_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=3, stride=1, pad=1)
            self.conv5_2_CPM_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=3, stride=1, pad=1)
            self.conv5_3_CPM_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=3, stride=1, pad=1)
            self.conv5_4_CPM_L2 = L.Convolution2D(
                in_channels=128, out_channels=512, ksize=1, stride=1, pad=0)
            self.conv5_5_CPM_L2 = L.Convolution2D(
                in_channels=512, out_channels=19, ksize=1, stride=1, pad=0)

            # stage2
            self.Mconv1_stage2_L1 = L.Convolution2D(
                in_channels=185, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv2_stage2_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv3_stage2_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv4_stage2_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv5_stage2_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv6_stage2_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=1, stride=1, pad=0)
            self.Mconv7_stage2_L1 = L.Convolution2D(
                in_channels=128, out_channels=38, ksize=1, stride=1, pad=0)
            self.Mconv1_stage2_L2 = L.Convolution2D(
                in_channels=185, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv2_stage2_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv3_stage2_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv4_stage2_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv5_stage2_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv6_stage2_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=1, stride=1, pad=0)
            self.Mconv7_stage2_L2 = L.Convolution2D(
                in_channels=128, out_channels=19, ksize=1, stride=1, pad=0)

            # stage3
            self.Mconv1_stage3_L1 = L.Convolution2D(
                in_channels=185, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv2_stage3_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv3_stage3_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv4_stage3_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv5_stage3_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv6_stage3_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=1, stride=1, pad=0)
            self.Mconv7_stage3_L1 = L.Convolution2D(
                in_channels=128, out_channels=38, ksize=1, stride=1, pad=0)
            self.Mconv1_stage3_L2 = L.Convolution2D(
                in_channels=185, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv2_stage3_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv3_stage3_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv4_stage3_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv5_stage3_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv6_stage3_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=1, stride=1, pad=0)
            self.Mconv7_stage3_L2 = L.Convolution2D(
                in_channels=128, out_channels=19, ksize=1, stride=1, pad=0)

            # stage4
            self.Mconv1_stage4_L1 = L.Convolution2D(
                in_channels=185, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv2_stage4_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv3_stage4_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv4_stage4_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv5_stage4_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv6_stage4_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=1, stride=1, pad=0)
            self.Mconv7_stage4_L1 = L.Convolution2D(
                in_channels=128, out_channels=38, ksize=1, stride=1, pad=0)
            self.Mconv1_stage4_L2 = L.Convolution2D(
                in_channels=185, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv2_stage4_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv3_stage4_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv4_stage4_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv5_stage4_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv6_stage4_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=1, stride=1, pad=0)
            self.Mconv7_stage4_L2 = L.Convolution2D(
                in_channels=128, out_channels=19, ksize=1, stride=1, pad=0)

            # stage5
            self.Mconv1_stage5_L1 = L.Convolution2D(
                in_channels=185, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv2_stage5_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv3_stage5_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv4_stage5_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv5_stage5_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv6_stage5_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=1, stride=1, pad=0)
            self.Mconv7_stage5_L1 = L.Convolution2D(
                in_channels=128, out_channels=38, ksize=1, stride=1, pad=0)
            self.Mconv1_stage5_L2 = L.Convolution2D(
                in_channels=185, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv2_stage5_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv3_stage5_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv4_stage5_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv5_stage5_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv6_stage5_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=1, stride=1, pad=0)
            self.Mconv7_stage5_L2 = L.Convolution2D(
                in_channels=128, out_channels=19, ksize=1, stride=1, pad=0)

            # stage6
            self.Mconv1_stage6_L1 = L.Convolution2D(
                in_channels=185, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv2_stage6_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv3_stage6_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv4_stage6_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv5_stage6_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv6_stage6_L1 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=1, stride=1, pad=0)
            self.Mconv7_stage6_L1 = L.Convolution2D(
                in_channels=128, out_channels=38, ksize=1, stride=1, pad=0)
            self.Mconv1_stage6_L2 = L.Convolution2D(
                in_channels=185, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv2_stage6_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv3_stage6_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv4_stage6_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv5_stage6_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=7, stride=1, pad=3)
            self.Mconv6_stage6_L2 = L.Convolution2D(
                in_channels=128, out_channels=128, ksize=1, stride=1, pad=0)
            self.Mconv7_stage6_L2 = L.Convolution2D(
                in_channels=128, out_channels=19, ksize=1, stride=1, pad=0)

            if pretrained_model in models.keys():
                data_dir = chainer.dataset.get_dataset_directory('openpose/pose')
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
        pafs = []

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
        h = F.relu(self.conv4_3_CPM(h))
        h = F.relu(self.conv4_4_CPM(h))
        feature_map = h

        # stage1
        h1 = F.relu(self.conv5_1_CPM_L1(feature_map))  # branch1
        h1 = F.relu(self.conv5_2_CPM_L1(h1))
        h1 = F.relu(self.conv5_3_CPM_L1(h1))
        h1 = F.relu(self.conv5_4_CPM_L1(h1))
        h1 = self.conv5_5_CPM_L1(h1)
        h2 = F.relu(self.conv5_1_CPM_L2(feature_map))  # branch2
        h2 = F.relu(self.conv5_2_CPM_L2(h2))
        h2 = F.relu(self.conv5_3_CPM_L2(h2))
        h2 = F.relu(self.conv5_4_CPM_L2(h2))
        h2 = self.conv5_5_CPM_L2(h2)
        pafs.append(h1)
        heatmaps.append(h2)

        # stage2
        h = F.concat((h1, h2, feature_map), axis=1)  # channel concat
        h1 = F.relu(self.Mconv1_stage2_L1(h))  # branch1
        h1 = F.relu(self.Mconv2_stage2_L1(h1))
        h1 = F.relu(self.Mconv3_stage2_L1(h1))
        h1 = F.relu(self.Mconv4_stage2_L1(h1))
        h1 = F.relu(self.Mconv5_stage2_L1(h1))
        h1 = F.relu(self.Mconv6_stage2_L1(h1))
        h1 = self.Mconv7_stage2_L1(h1)
        h2 = F.relu(self.Mconv1_stage2_L2(h))  # branch2
        h2 = F.relu(self.Mconv2_stage2_L2(h2))
        h2 = F.relu(self.Mconv3_stage2_L2(h2))
        h2 = F.relu(self.Mconv4_stage2_L2(h2))
        h2 = F.relu(self.Mconv5_stage2_L2(h2))
        h2 = F.relu(self.Mconv6_stage2_L2(h2))
        h2 = self.Mconv7_stage2_L2(h2)
        pafs.append(h1)
        heatmaps.append(h2)

        # stage3
        h = F.concat((h1, h2, feature_map), axis=1)  # channel concat
        h1 = F.relu(self.Mconv1_stage3_L1(h))  # branch1
        h1 = F.relu(self.Mconv2_stage3_L1(h1))
        h1 = F.relu(self.Mconv3_stage3_L1(h1))
        h1 = F.relu(self.Mconv4_stage3_L1(h1))
        h1 = F.relu(self.Mconv5_stage3_L1(h1))
        h1 = F.relu(self.Mconv6_stage3_L1(h1))
        h1 = self.Mconv7_stage3_L1(h1)
        h2 = F.relu(self.Mconv1_stage3_L2(h))  # branch2
        h2 = F.relu(self.Mconv2_stage3_L2(h2))
        h2 = F.relu(self.Mconv3_stage3_L2(h2))
        h2 = F.relu(self.Mconv4_stage3_L2(h2))
        h2 = F.relu(self.Mconv5_stage3_L2(h2))
        h2 = F.relu(self.Mconv6_stage3_L2(h2))
        h2 = self.Mconv7_stage3_L2(h2)
        pafs.append(h1)
        heatmaps.append(h2)

        # stage4
        h = F.concat((h1, h2, feature_map), axis=1)  # channel concat
        h1 = F.relu(self.Mconv1_stage4_L1(h))  # branch1
        h1 = F.relu(self.Mconv2_stage4_L1(h1))
        h1 = F.relu(self.Mconv3_stage4_L1(h1))
        h1 = F.relu(self.Mconv4_stage4_L1(h1))
        h1 = F.relu(self.Mconv5_stage4_L1(h1))
        h1 = F.relu(self.Mconv6_stage4_L1(h1))
        h1 = self.Mconv7_stage4_L1(h1)
        h2 = F.relu(self.Mconv1_stage4_L2(h))  # branch2
        h2 = F.relu(self.Mconv2_stage4_L2(h2))
        h2 = F.relu(self.Mconv3_stage4_L2(h2))
        h2 = F.relu(self.Mconv4_stage4_L2(h2))
        h2 = F.relu(self.Mconv5_stage4_L2(h2))
        h2 = F.relu(self.Mconv6_stage4_L2(h2))
        h2 = self.Mconv7_stage4_L2(h2)
        pafs.append(h1)
        heatmaps.append(h2)

        # stage5
        h = F.concat((h1, h2, feature_map), axis=1)  # channel concat
        h1 = F.relu(self.Mconv1_stage5_L1(h))  # branch1
        h1 = F.relu(self.Mconv2_stage5_L1(h1))
        h1 = F.relu(self.Mconv3_stage5_L1(h1))
        h1 = F.relu(self.Mconv4_stage5_L1(h1))
        h1 = F.relu(self.Mconv5_stage5_L1(h1))
        h1 = F.relu(self.Mconv6_stage5_L1(h1))
        h1 = self.Mconv7_stage5_L1(h1)
        h2 = F.relu(self.Mconv1_stage5_L2(h))  # branch2
        h2 = F.relu(self.Mconv2_stage5_L2(h2))
        h2 = F.relu(self.Mconv3_stage5_L2(h2))
        h2 = F.relu(self.Mconv4_stage5_L2(h2))
        h2 = F.relu(self.Mconv5_stage5_L2(h2))
        h2 = F.relu(self.Mconv6_stage5_L2(h2))
        h2 = self.Mconv7_stage5_L2(h2)
        pafs.append(h1)
        heatmaps.append(h2)

        # stage6
        h = F.concat((h1, h2, feature_map), axis=1)  # channel concat
        h1 = F.relu(self.Mconv1_stage6_L1(h))  # branch1
        h1 = F.relu(self.Mconv2_stage6_L1(h1))
        h1 = F.relu(self.Mconv3_stage6_L1(h1))
        h1 = F.relu(self.Mconv4_stage6_L1(h1))
        h1 = F.relu(self.Mconv5_stage6_L1(h1))
        h1 = F.relu(self.Mconv6_stage6_L1(h1))
        h1 = self.Mconv7_stage6_L1(h1)
        h2 = F.relu(self.Mconv1_stage6_L2(h))  # branch2
        h2 = F.relu(self.Mconv2_stage6_L2(h2))
        h2 = F.relu(self.Mconv3_stage6_L2(h2))
        h2 = F.relu(self.Mconv4_stage6_L2(h2))
        h2 = F.relu(self.Mconv5_stage6_L2(h2))
        h2 = F.relu(self.Mconv6_stage6_L2(h2))
        h2 = self.Mconv7_stage6_L2(h2)
        pafs.append(h1)
        heatmaps.append(h2)

        return pafs, heatmaps


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
    chainer_model = PoseNet(pretrained_model=None)
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
