from jsk_recognition_utils.chainermodels import alexnet
from jsk_recognition_utils.chainermodels import alexnet_batch_normalization
from jsk_recognition_utils.chainermodels import deep_sort_net
from jsk_recognition_utils.chainermodels import fcn8s_depth_predicition
from jsk_recognition_utils.chainermodels import vgg16
from jsk_recognition_utils.chainermodels import vgg16_batch_normalization
from jsk_recognition_utils.chainermodels import vgg16_fast_rcnn
from jsk_recognition_utils.chainermodels import vgg_cnn_m_1024
from jsk_recognition_utils.chainermodels import resnet

# AlexNet Object Recognition Network
AlexNet = alexnet.AlexNet
AlexNetBatchNormalization = alexnet_batch_normalization.AlexNetBatchNormalization  # NOQA

# VGG16 Object Recognition Network
VGG16 = vgg16.VGG16
VGG16BatchNormalization = vgg16_batch_normalization.VGG16BatchNormalization

# FastRCNN
VGG16FastRCNN = vgg16_fast_rcnn.VGG16FastRCNN
VGG_CNN_M_1024 = vgg_cnn_m_1024.VGG_CNN_M_1024

# ResNet
ResNet50 = resnet.ResNet50
ResNet50Feature = resnet.ResNet50Feature
ResNet101 = resnet.ResNet101
ResNet101Feature = resnet.ResNet101Feature
ResNet152 = resnet.ResNet152
ResNet152Feature = resnet.ResNet152Feature

# FCN Depth Prediction Network
FCN8sDepthPrediction = fcn8s_depth_predicition.FCN8sDepthPrediction
FCN8sDepthPredictionConcatFirst = fcn8s_depth_predicition.FCN8sDepthPredictionConcatFirst  # NOQA
