#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


import itertools, pkg_resources, sys
from distutils.version import LooseVersion
if LooseVersion(pkg_resources.get_distribution("chainer").version) >= LooseVersion('7.0.0') and \
        sys.version_info.major == 2:
    print('''Please install chainer < 7.0.0:

    sudo pip install chainer==6.7.0

c.f https://github.com/jsk-ros-pkg/jsk_recognition/pull/2485
''', file=sys.stderr)
    sys.exit(1)
if [p for p in list(itertools.chain(*[pkg_resources.find_distributions(_) for _ in sys.path])) if "cupy-" in p.project_name or "cupy" == p.project_name ] == []:
    print('''Please install CuPy

    sudo pip install cupy-cuda[your cuda version]
i.e.
    sudo pip install cupy-cuda91

''', file=sys.stderr)
    # sys.exit(1)
import chainer
from chainer import cuda
import chainer.serializers as S
from chainer import Variable
from distutils.version import LooseVersion
import numpy as np
import skimage.transform

import cv_bridge
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_utils.chainermodels import VGG16
from jsk_recognition_utils.chainermodels import VGG16BatchNormalization
from jsk_topic_tools import ConnectionBasedTransport
from jsk_topic_tools.log_utils import logerr_throttle
import message_filters
import rospy
from sensor_msgs.msg import Image

from jsk_perception.vgg16_object_recognition import VGG16ObjectRecognition



if __name__ == '__main__':
    rospy.init_node('vgg16_object_recognition')
    app = VGG16ObjectRecognition()
    rospy.spin()
