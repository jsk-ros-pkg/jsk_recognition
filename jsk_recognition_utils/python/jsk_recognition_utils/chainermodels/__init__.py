from jsk_recognition_utils.chainermodels import alexnet
from jsk_recognition_utils.chainermodels import alexnet_batch_normalization
from jsk_recognition_utils.chainermodels import deep_sort_net
from jsk_recognition_utils.chainermodels import fcn8s_depth_predicition
from jsk_recognition_utils.chainermodels import vgg16
from jsk_recognition_utils.chainermodels import vgg16_batch_normalization
from jsk_recognition_utils.chainermodels import vgg16_fast_rcnn
from jsk_recognition_utils.chainermodels import vgg_cnn_m_1024

# AlexNet Object Recognition Network
AlexNet = alexnet.AlexNet
AlexNetBatchNormalization = alexnet_batch_normalization.AlexNetBatchNormalization  # NOQA

# VGG16 Object Recognition Network
VGG16 = vgg16.VGG16
VGG16BatchNormalization = vgg16_batch_normalization.VGG16BatchNormalization

# FastRCNN
VGG16FastRCNN = vgg16_fast_rcnn.VGG16FastRCNN
VGG_CNN_M_1024 = vgg_cnn_m_1024.VGG_CNN_M_1024

# ResNet (chainermodels/resnet/) is not exported, as its original work
# yasunorikudo/chainer-ResNet has no license; see LICENSE.

# FCN Depth Prediction Network
FCN8sDepthPrediction = fcn8s_depth_predicition.FCN8sDepthPrediction
FCN8sDepthPredictionConcatFirst = fcn8s_depth_predicition.FCN8sDepthPredictionConcatFirst  # NOQA
