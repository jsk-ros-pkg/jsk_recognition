#!/usr/bin/env python

# classify spectrogram using neural networks

import chainer
from chainer import cuda
from chainer_modules import nin

from cv_bridge import CvBridge
import numpy as np
import os.path as osp
from PIL import Image as Image_
import rospy
import rospkg
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ClassifySpectrogramROS:
    def __init__(self):
        rospy.init_node('classify_spectrogram_ros')
        archs = {  # only NIN is availale now
            # 'alex': alex.Alex,
            # 'googlenet': googlenet.GoogLeNet,
            # 'googlenetbn': googlenetbn.GoogLeNetBN,
            'nin': nin.NIN,
            # 'resnet50': resnet50.ResNet50,
            # 'resnext50': resnext50.ResNeXt50,
        }

        self.gpu = 0
        device = chainer.cuda.get_device(self.gpu)  # for python2, gpu number is 0

        print('Device: {}'.format(device))
        print('Dtype: {}'.format(chainer.config.dtype))
        print('')

        # Initialize the model to train
        rospack = rospkg.RosPack()
        n_class_file = osp.join(
            rospack.get_path('sound_classification'),
            'train_data', 'dataset', 'n_class.txt')
        n_class = 0
        self.classes = []
        with open(n_class_file, mode='r') as f:
            for row in f:
                self.classes.append(row.strip())
                n_class += 1
        self.model = archs['nin'](n_class=n_class)
        initmodel = rospy.get_param('~model')
        print('Load model from {}'.format(initmodel))
        chainer.serializers.load_npz(initmodel, self.model)
        self.model.to_device(device)
        device.use()

        # Load the mean file
        mean_file_path = osp.join(rospack.get_path('sound_classification'),
                                  'train_data', 'dataset', 'mean_of_dataset.png')
        self.mean = np.array(Image_.open(mean_file_path), np.float32).transpose(
            (2, 0, 1))  # (256,256,3) -> (3,256,256), rgb

        # Set up an optimizer
        optimizer = chainer.optimizers.MomentumSGD(lr=0.01, momentum=0.9)
        optimizer.setup(self.model)

        # subscriber and publisher
        self.hit_sub = rospy.Subscriber(
            '/microphone/hit_spectrogram', Image, self.hit_cb)
        self.pub = rospy.Publisher(
            '/object_class_by_image', String, queue_size=1)

        self.bridge = CvBridge()

    def hit_cb(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        with chainer.using_config('train', False), \
             chainer.no_backprop_mode():
            x_data = np.array(Image_.fromarray(cv_image).resize((256, 256))).astype(np.float32)
            x_data = x_data.transpose(
                (2, 0, 1))[[2, 1, 0], :, :]  # (256,256,3) -> (3,256,256), bgr -> rgb
            mean = self.mean.astype(np.float32)
            x_data -= mean
            x_data *= (1.0 / 255.0)  # Scale to [0, 1.0]
            # fowarding once
            x_data = cuda.to_gpu(x_data[None], device=self.gpu)
            x_data = chainer.Variable(x_data)
            ret = self.model.forward_for_test(x_data)
            ret = cuda.to_cpu(ret.data)[0]
        msg = String()
        msg.data = self.classes[np.argmax(ret)]
        self.pub.publish(msg)
        rospy.sleep(0.5)
        msg.data = 'no object'
        self.pub.publish(msg)


if __name__ == '__main__':
    csir = ClassifySpectrogramROS()
    rospy.spin()
