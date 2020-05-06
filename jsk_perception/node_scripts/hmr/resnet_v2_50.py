from __future__ import print_function

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
from chainer import initializers
import chainer.links as L


class BottleNeckA(chainer.Chain):

    def __init__(self, in_size, ch, out_size, stride=2):
        super(BottleNeckA, self).__init__()
        initialW = initializers.HeNormal()
        self.stride = stride

        with self.init_scope():
            self.preact = L.BatchNormalization(in_size)
            self.conv1 = L.Convolution2D(
                in_size, ch, 1, stride, 0, initialW=initialW, nobias=True)
            self.bn1 = L.BatchNormalization(ch)
            self.conv2 = L.Convolution2D(
                ch, ch, 3, stride, 1, initialW=initialW, nobias=True)
            self.bn2 = L.BatchNormalization(ch)
            self.conv3 = L.Convolution2D(
                ch, out_size, 1, 1, 0, initialW=initialW, nobias=False)
            self.conv4 = L.Convolution2D(
                in_size, out_size, 1, stride, 0,
                initialW=initialW, nobias=False)

    def __call__(self, x):
        preact = F.relu(self.preact(x))
        shortcut = self.conv4(preact)
        h = self.conv1(preact)
        h = F.relu(self.bn1(h))
        h = F.relu(self.bn2(self.conv2(h)))
        h = self.conv3(h)
        return shortcut + h


class BottleNeckB(chainer.Chain):

    def __init__(self, in_size, ch, stride=2):
        super(BottleNeckB, self).__init__()
        initialW = initializers.HeNormal()
        self.stride = stride

        with self.init_scope():
            self.preact = L.BatchNormalization(in_size)
            self.conv1 = L.Convolution2D(
                in_size, ch, 1, 1, 0, initialW=initialW, nobias=True)
            self.bn1 = L.BatchNormalization(ch)
            self.conv2 = L.Convolution2D(
                ch, ch, 3, self.stride, 1, initialW=initialW, nobias=True)
            self.bn2 = L.BatchNormalization(ch)
            self.conv3 = L.Convolution2D(
                ch, in_size, 1, 1, 0, initialW=initialW, nobias=False)

    def __call__(self, x):
        preact = F.relu(self.preact(x))
        if self.stride == 1:
            shortcut = x
        else:
            shortcut = F.max_pooling_2d(
                x, 1, stride=self.stride, cover_all=False)
        h = self.conv1(preact)
        h = F.relu(self.bn1(h))
        h = F.relu(self.bn2(self.conv2(h)))
        h = self.conv3(h)
        return shortcut + h


class Block(chainer.ChainList):

    def __init__(self, layer, in_size, ch, out_size, stride=2):
        super(Block, self).__init__()
        self.add_link(BottleNeckA(in_size, ch, out_size, 1))
        for i in range(layer - 2):
            self.add_link(BottleNeckB(out_size, ch, stride=1))
        self.add_link(BottleNeckB(out_size, ch, stride=stride))

    def __call__(self, x):
        for f in self.children():
            x = f(x)
        return x


class ResNet_v2_50(chainer.Chain):

    insize = 224

    def __init__(self):
        super(ResNet_v2_50, self).__init__()
        with self.init_scope():
            self.conv1 = L.Convolution2D(
                3, 64, 7, 2, 3, initialW=initializers.HeNormal())
            self.res2 = Block(3, 64, 64, 256, stride=2)
            self.res3 = Block(4, 256, 128, 512, stride=2)
            self.res4 = Block(6, 512, 256, 1024, stride=2)
            self.res5 = Block(3, 1024, 512, 2048, stride=1)
            self.postnorm = L.BatchNormalization(2048)

    def __call__(self, x):
        h = self.conv1(x)
        h = F.max_pooling_2d(h, 3, stride=2)
        h = self.res2(h)
        h = self.res3(h)
        h = self.res4(h)
        h = self.res5(h)
        h = self.postnorm(h)
        h = F.relu(h)
        h = F.average_pooling_2d(h, 7, stride=1)
        return h
