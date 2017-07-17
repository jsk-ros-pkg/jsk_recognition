#!/usr/bin/env python


import cv2
import numpy as np

import warnings

import chainer
from chainer import cuda
from chainer.datasets import TupleDataset
import chainer.functions as F
import chainer.links as L
from chainer.initializers import constant
from chainer.initializers import normal

from chainercv.transforms import center_crop
from chainercv.transforms import scale
from chainercv.utils.iterator import apply_prediction_to_iterator
from chainercv.utils import download_model

from jsk_arc2017_common.in_hand_recognition.resnet_pca import ResNet50PCA
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from jsk_recognition_msgs.msg import ClassificationResult

from cv_bridge import CvBridge
from jsk_topic_tools import ConnectionBasedTransport


def calc_nearest_neighbors(query_feats, db_feats, k):
    """Calculate K Nearest Neighbor

    """
    query_feats = np.array(list(query_feats))
    db_feats = np.array(list(db_feats))
    labels = np.arange(len(db_feats))
    if cv2.__version__.startswith('2.4'):
        knn = cv2.KNearest()
        knn.train(db_feats, labels)
        _, _, indices, dists = knn.find_nearest(query_feats, k)
    elif cv2.__version__ == '3.2.0':
        knn = cv2.ml.KNearest_create()
        knn.train(db_feats, cv2.ml.ROW_SAMPLE, labels)
        _, _, indices, dists = knn.findNearest(query_feats, k)
    else:
        raise ValueError('only version OpenCV with version 2.4.* and 3.2 '
                         'are supported.')
    return indices.astype(np.int32), dists


class BuildingBlock(chainer.Chain):

    """A building block that consists of several Bottleneck layers.

    Args:
        n_layer (int): Number of layers used in the building block.
        in_channels (int): Number of channels of input arrays.
        mid_channels (int): Number of channels of intermediate arrays.
        out_channels (int): Number of channels of output arrays.
        stride (int or tuple of ints): Stride of filter application.
        initialW (4-D array): Initial weight value used in
            the convolutional layers.
    """

    def __init__(self, n_layer, in_channels, mid_channels,
                 out_channels, stride, initialW=None):
        super(BuildingBlock, self).__init__()
        with self.init_scope():
            self.a = BottleneckA(
                in_channels, mid_channels, out_channels, stride, initialW)
            self._forward = ["a"]
            for i in range(n_layer - 1):
                name = 'b{}'.format(i + 1)
                bottleneck = BottleneckB(out_channels, mid_channels, initialW)
                setattr(self, name, bottleneck)
                self._forward.append(name)

    def __call__(self, x):
        for name in self._forward:
            l = getattr(self, name)
            x = l(x)
        return x


class BottleneckA(chainer.Chain):

    """A bottleneck layer that reduces the resolution of the feature map.

    Args:
        in_channels (int): Number of channels of input arrays.
        mid_channels (int): Number of channels of intermediate arrays.
        out_channels (int): Number of channels of output arrays.
        stride (int or tuple of ints): Stride of filter application.
        initialW (4-D array): Initial weight value used in
            the convolutional layers.
    """

    def __init__(self, in_channels, mid_channels, out_channels,
                 stride=2, initialW=None):
        super(BottleneckA, self).__init__()
        with self.init_scope():
            self.conv1 = L.Convolution2D(
                in_channels, mid_channels, 1, stride, 0, initialW=initialW,
                nobias=True)
            self.bn1 = L.BatchNormalization(mid_channels)
            self.conv2 = L.Convolution2D(
                mid_channels, mid_channels, 3, 1, 1, initialW=initialW,
                nobias=True)
            self.bn2 = L.BatchNormalization(mid_channels)
            self.conv3 = L.Convolution2D(
                mid_channels, out_channels, 1, 1, 0, initialW=initialW,
                nobias=True)
            self.bn3 = L.BatchNormalization(out_channels)
            self.conv4 = L.Convolution2D(
                in_channels, out_channels, 1, stride, 0, initialW=initialW,
                nobias=True)
            self.bn4 = L.BatchNormalization(out_channels)

    def __call__(self, x):
        h1 = F.relu(self.bn1(self.conv1(x)))
        h1 = F.relu(self.bn2(self.conv2(h1)))
        h1 = self.bn3(self.conv3(h1))
        h2 = self.bn4(self.conv4(x))
        return F.relu(h1 + h2)


class BottleneckB(chainer.Chain):

    """A bottleneck layer that maintains the resolution of the feature map.

    Args:
        in_channels (int): Number of channels of input and output arrays.
        mid_channels (int): Number of channels of intermediate arrays.
        initialW (4-D array): Initial weight value used in
            the convolutional layers.
    """

    def __init__(self, in_channels, mid_channels, initialW=None):
        super(BottleneckB, self).__init__()
        with self.init_scope():
            self.conv1 = L.Convolution2D(
                in_channels, mid_channels, 1, 1, 0, initialW=initialW,
                nobias=True)
            self.bn1 = L.BatchNormalization(mid_channels)
            self.conv2 = L.Convolution2D(
                mid_channels, mid_channels, 3, 1, 1, initialW=initialW,
                nobias=True)
            self.bn2 = L.BatchNormalization(mid_channels)
            self.conv3 = L.Convolution2D(
                mid_channels, in_channels, 1, 1, 0, initialW=initialW,
                nobias=True)
            self.bn3 = L.BatchNormalization(in_channels)

    def __call__(self, x):
        h = F.relu(self.bn1(self.conv1(x)))
        h = F.relu(self.bn2(self.conv2(h)))
        h = self.bn3(self.conv3(h))
        return F.relu(h + x)


def _global_average_pooling_2d(x):
    n, channel, rows, cols = x.data.shape
    h = F.average_pooling_2d(x, (rows, cols), stride=1)
    h = h.reshape(n, channel)
    return h


class ResNet50(chainer.Chain):

    _models = {
        'imagenet': {
            'n_class': 1000,
            'url': 'https://github.com/yuyu2172/chainer-tools/releases/'
            'download/v0.0.1/resnet50_06_19.npz'
        }
    }

    def __init__(self, pretrained_model=None):
        super(ResNet50, self).__init__()
        kwargs = {}
        with self.init_scope():
            self.conv1 = L.Convolution2D(3, 64, 7, 2, 3, **kwargs)
            self.bn1 = L.BatchNormalization(64)
            self.res2 = BuildingBlock(3, 64, 64, 256, 1, **kwargs)
            self.res3 = BuildingBlock(4, 256, 128, 512, 2, **kwargs)
            self.res4 = BuildingBlock(6, 512, 256, 1024, 2, **kwargs)
            self.res5 = BuildingBlock(3, 1024, 512, 2048, 2, **kwargs)

        if pretrained_model in self._models:
            path = download_model(self._models[pretrained_model]['url'])
            chainer.serializers.load_npz(path, self)
        elif pretrained_model:
            chainer.serializers.load_npz(pretrained_model, self)

    def __call__(self, x):
        h = self.conv1(x)
        h = self.bn1(h)
        h = F.relu(h)
        h = F.max_pooling_2d(h, ksize=3, stride=2)
        h = self.res2(h)
        h = self.res3(h)
        h = self.res4(h)
        h = self.res5(h)
        h = _global_average_pooling_2d(h)
        return h
        

def _global_average_pooling_2d(x):
    n, channel, rows, cols = x.data.shape
    h = F.average_pooling_2d(x, (rows, cols), stride=1)
    h = h.reshape(n, channel)
    return h


# RGB order
_imagenet_mean = np.array(
    [123.68, 116.779, 103.939], dtype=np.float32)[:, np.newaxis, np.newaxis]


class RetrievalPredictor(chainer.Chain):

    def __init__(self, extractor, mean=_imagenet_mean,
                 size=(224, 224), scale_size=256, k=5):
        super(RetrievalPredictor, self).__init__()
        self.mean = mean
        self.scale_size = scale_size
        self.size = size
        self.k = 5

        with self.init_scope():
            self.extractor = extractor

        self.db_features = None

    def _prepare(self, img):
        """Prepare an image to be used for prediction.

        Args:
            img (~numpy.ndarray): An image. This is in CHW and RGB format.
                The range of its value is :math:`[0, 255]`.

        Returns:
            ~numpy.ndarray:
            A preprocessed image.

        """
        img = scale(img, size=self.scale_size)
        img = center_crop(img, self.size)
        img = img - self.mean

        return img

    def extract(self, imgs):
        """Extract features from raw images

        Args:
            imgs (~numpy.ndarray): Batch of images. An image is in CHW and RGB
                format.
                The range of its value is :math:`[0, 255]`.

        """
        imgs = [self._prepare(img) for img in imgs]
        imgs = self.xp.asarray(imgs).reshape(-1, 3, 224, 224)

        with chainer.function.no_backprop_mode():
            with chainer.using_config('train', False):
                imgs = chainer.Variable(imgs)
                activations = self.extractor(imgs)

        output = cuda.to_cpu(activations.data)
        return output

    def load_db(self, it):
        """Set database features.

        Args:
            it (chainer.dataset.Iterator)

        """
        if it._repeat:
            raise ValueError('This does not accept infinite length iterator. '
                             'Please set `repeat` option of iterator to False.')
        if it._shuffle:
            warnings.warn('`shuffle` is True. Is this OK?')

        imgs, (db_features,), (db_labels,) = apply_prediction_to_iterator(
            self.extract, it)
        del imgs
        self.db_features = np.array(list(db_features))
        self.db_labels = np.array(list(db_labels))

    def predict(self, imgs):
        """Find K nearest negihbors of query images from the database.

        Args
            imgs: batch of query images.

        """
        query_features = self.extract(imgs)

        if self.db_features is None:
            raise ValueError('Please prepare database features. '
                             'This can be done with method `load_db`.')

        top_k, _ = calc_nearest_neighbors(
            query_features, self.db_features, self.k)
        return top_k


class InHandRecognitionNode(ConnectionBasedTransport):

    """In hand recognition using an image retrieval algorithm.

    Two rosparams are paths to database images and labels.

    * ~imgs_path: Path to a numpy.ndarray file. It contains batch of images,
        and its shape is (B, 3, H, W).
    * ~labels_path: Path to a numpy.ndarray file. It contains batch of labels,
        and its shape is (B,).

    The ClassificationResult message contains class labels that corresponds to
    the class labels in the database.

    """

    def __init__(self):
        super(InHandRecognitionNode, self).__init__()
        gpu = rospy.get_param('~gpu', -1)
        imgs_path = rospy.get_param('~images_path')
        labels_path = rospy.get_param('~labels_path')

        base_model = ResNet50(pretrained_model='imagenet')
        self.model = RetrievalPredictor(base_model, k=3)
        if gpu >= 0:
            self.model.to_gpu(gpu)
            chainer.cuda.get_device(gpu).use()
        imgs = np.load(imgs_path)[:1]
        labels = np.load(labels_path)[:1]
        it = chainer.iterators.SerialIterator(
            TupleDataset(imgs, labels), batch_size=1,
            shuffle=False, repeat=False)
        self.model.load_db(it)

        self.pub =self.advertise(
            '~classification', ClassificationResult, queue_size=10)
        self.bridge = CvBridge()

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Image, self.callback)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, img_msg):
        # Subscribe results
        img = self.bridge.imgmsg_to_cv2(img_msg)
        # TODO: Make sure that this image is RGB
        img = img.transpose(2, 0, 1)

        # Predictions
        pred_indices = self.model.predict([img])[0]  # (K,)
        pred_labels = self.model.db_labels[pred_indices]  # (K,)
        probs = np.linspace(1, 0, len(pred_labels) + 1)[:-1]

        # Publish results
        msg = ClassificationResult(
            labels=pred_labels,
            label_proba=probs,
            classifier='in_hand_recognition'
        )
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('in_hand_recognition')
    node = InHandRecognitionNode()
    rospy.spin()
