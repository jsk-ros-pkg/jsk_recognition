import chainer
import chainer.functions as F
import chainer.links as L
from chainer import initializers


class BlockA(chainer.Chain):

    def __init__(self, in_size, out_size, stride=1, activation=F.elu,
                 is_first=False, pad=1, projection_pad=0, flag=False):
        super(BlockA, self).__init__()
        initialW = initializers.HeNormal()

        self.in_size = in_size
        self.out_size = out_size
        self.is_first = is_first
        self.activation = activation
        self.flag = flag
        with self.init_scope():
            if is_first is False:
                self.bn0 = L.BatchNormalization(in_size)
            if flag is False:
                self.conv1 = L.Convolution2D(
                    in_size, out_size, 3, stride, pad,
                    initialW=initialW, nobias=True)
            else:
                self.conv1 = L.Convolution2D(
                    in_size, out_size, 3, stride, 0,
                    initialW=initialW, nobias=True)
            self.bn1 = L.BatchNormalization(out_size)
            self.conv2 = L.Convolution2D(
                out_size, out_size, 3, 1,
                pad, initialW=initialW, nobias=False)
            if in_size != out_size:
                if out_size != 2 * in_size:
                    raise ValueError('out_size should be two 2 * in_size ',
                                     '{} != {}'.format(out_size, 2 * in_size))
                self.projection = L.Convolution2D(
                    in_size, out_size,
                    1, stride, projection_pad, nobias=True)

    def __call__(self, x):
        batchsize, channel, height, width = x.shape
        if self.is_first:
            h = x
        else:
            h = self.activation(self.bn0(x))
        if self.flag:
            # for (top, bottom, left, right) padding
            #     (0, 1, 0, 1)
            h = F.concat([F.concat([
                h,
                self.xp.zeros((batchsize, channel, 1, width), 'f')], axis=2),
                self.xp.zeros((
                    batchsize, channel, height + 1, 1), 'f')],
                axis=3)
            h = self.activation(self.bn1(self.conv1(h)))
        else:
            h = self.activation(self.bn1(self.conv1(h)))
        h = F.dropout(h, ratio=0.6)
        h = self.conv2(h)
        if self.in_size != self.out_size:
            return self.projection(x) + h
        else:
            return x + h


class DeepSortFeatureExtractor(chainer.Chain):

    def __init__(self):
        super(DeepSortFeatureExtractor, self).__init__()

        with self.init_scope():
            self.conv1_1 = L.Convolution2D(
                3, 32, 3, 1, 1, nobias=True)
            self.bn1 = L.BatchNormalization(32)
            self.conv1_2 = L.Convolution2D(
                32, 32, 3, 1, 1, nobias=True)
            self.bn2 = L.BatchNormalization(32)
            self.conv2_1 = BlockA(32, 32, stride=1, is_first=True)
            self.conv2_3 = BlockA(32, 32, stride=1)
            self.conv3_1 = BlockA(
                32, 64, stride=2, pad=1, projection_pad=0)
            self.conv3_3 = BlockA(64, 64, stride=1)
            self.conv4_1 = BlockA(
                64, 128, stride=2, pad=1, projection_pad=0, flag=True)
            self.conv4_3 = BlockA(128, 128, stride=1)
            self.fc1 = L.Linear(16384, 128, nobias=True)
            self.fc1_bn = L.BatchNormalization(128)

            self.ball = L.BatchNormalization(128)
            self.mean_vectors = chainer.Parameter(0, shape=[128, 1501])
            self.scale = chainer.Parameter(0, shape=[1501])

    def __call__(self, x):
        # x.shape == (batchsize, 3, 128, 64)
        batchsize = x.shape[0]
        h = F.elu(self.bn1(self.conv1_1(x)))
        h = F.elu(self.bn2(self.conv1_2(h)))
        h = F.max_pooling_2d(h, 3, 2, cover_all=False)
        h = self.conv2_1(h)
        h = self.conv2_3(h)
        h = self.conv3_1(h)
        h = self.conv3_3(h)
        h = self.conv4_1(h)
        h = self.conv4_3(h)

        h = h.reshape(batchsize, -1)
        h = F.dropout(h, ratio=0.6)
        h = F.elu(self.fc1_bn(self.fc1(h)))

        # Features in rows, normalize axis 1.
        weights = self.mean_vectors
        features = self.ball(h)
        features = F.normalize(features, eps=1e-8)
        scale = F.softplus(self.scale)
        normalized_weight = F.normalize(weights, axis=0, eps=1e-8)
        logits = F.tile(scale[None, ], (batchsize, 1)) * \
            F.matmul(features, normalized_weight)
        return logits
