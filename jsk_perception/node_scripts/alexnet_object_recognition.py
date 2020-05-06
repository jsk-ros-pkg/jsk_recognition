#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
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
import chainer.serializers as S
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_utils.chainermodels import AlexNet
from jsk_recognition_utils.chainermodels import AlexNetBatchNormalization
import rospy
from sensor_msgs.msg import Image
from vgg16_object_recognition import VGG16ObjectRecognition


class AlexNetObjectRecognition(VGG16ObjectRecognition):

    def __init__(self):
        super(VGG16ObjectRecognition, self).__init__()
        self.insize = 227
        self.gpu = rospy.get_param('~gpu', -1)
        self.target_names = rospy.get_param('~target_names')
        self.model_name = rospy.get_param('~model_name')
        if self.model_name == 'alexnet':
            self.model = AlexNet(n_class=len(self.target_names))
        elif self.model_name == 'alexnet_batch_normalization':
            self.model = AlexNetBatchNormalization(
                n_class=len(self.target_names))
        else:
            rospy.logerr('Unsupported ~model_name: {0}'
                         .format(self.model_name))
        model_file = rospy.get_param('~model_file')
        S.load_hdf5(model_file, self.model)
        if self.gpu != -1:
            self.model.to_gpu(self.gpu)
        self.pub = self.advertise('~output', ClassificationResult,
                                  queue_size=1)
        self.pub_input = self.advertise(
            '~debug/net_input', Image, queue_size=1)


if __name__ == '__main__':
    rospy.init_node('alexnet_object_recognition')
    app = AlexNetObjectRecognition()
    rospy.spin()
