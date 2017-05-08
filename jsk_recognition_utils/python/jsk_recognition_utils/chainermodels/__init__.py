from jsk_recognition_utils.chainermodels import alex
from jsk_recognition_utils.chainermodels import alex_batch_normalization
from jsk_recognition_utils.chainermodels import vgg16
from jsk_recognition_utils.chainermodels import vgg16_batch_normalization
from jsk_recognition_utils.chainermodels import vgg16_fast_rcnn
from jsk_recognition_utils.chainermodels import vgg_cnn_m_1024

# Alex Object Recognition Network
Alex = alex.Alex
AlexBatchNormalization = alex_batch_normalization.AlexBatchNormalization

# VGG16 Object Recognition Network
VGG16 = vgg16.VGG16
VGG16BatchNormalization = vgg16_batch_normalization.VGG16BatchNormalization

# FastRCNN
VGG16FastRCNN = vgg16_fast_rcnn.VGG16FastRCNN
VGG_CNN_M_1024 = vgg_cnn_m_1024.VGG_CNN_M_1024
