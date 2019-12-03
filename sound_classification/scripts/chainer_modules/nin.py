import chainer
import chainer.functions as F
import chainer.initializers as I
import chainer.links as L

# mainly copied from https://github.com/chainer/chainer/blob/master/examples/imagenet/nin.py


class NIN(chainer.Chain):

    """Network-in-Network example model."""

    insize = 227

    def __init__(self, n_class=1000):  # 1000 is for ImageNet
        super(NIN, self).__init__()
        conv_init = I.HeNormal()  # MSRA scaling
        self.n_class = n_class

        with self.init_scope():
            self.mlpconv1 = L.MLPConvolution2D(
                None, (96, 96, 96), 11, stride=4, conv_init=conv_init)
            self.mlpconv2 = L.MLPConvolution2D(
                None, (256, 256, 256), 5, pad=2, conv_init=conv_init)
            self.mlpconv3 = L.MLPConvolution2D(
                None, (384, 384, 384), 3, pad=1, conv_init=conv_init)
            self.mlpconv4 = L.MLPConvolution2D(
                None, (1024, 1024, self.n_class), 3, pad=1, conv_init=conv_init)

    def forward(self, x, t):
        h = F.max_pooling_2d(F.relu(self.mlpconv1(x)), 3, stride=2)
        h = F.max_pooling_2d(F.relu(self.mlpconv2(h)), 3, stride=2)
        h = F.max_pooling_2d(F.relu(self.mlpconv3(h)), 3, stride=2)
        h = self.mlpconv4(F.dropout(h))
        h = F.reshape(F.average_pooling_2d(h, 6), (len(x), self.n_class))
        loss = F.softmax_cross_entropy(h, t)

        chainer.report({'loss': loss, 'accuracy': F.accuracy(h, t)}, self)
        return loss

    def forward_for_test(self, x):
        h = F.max_pooling_2d(F.relu(self.mlpconv1(x)), 3, stride=2)
        h = F.max_pooling_2d(F.relu(self.mlpconv2(h)), 3, stride=2)
        h = F.max_pooling_2d(F.relu(self.mlpconv3(h)), 3, stride=2)
        h = self.mlpconv4(F.dropout(h))
        h = F.reshape(F.average_pooling_2d(h, 6), (len(x), self.n_class))
        h = F.softmax(h)
        return h
