#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
import jsk_recognition_msgs.msg
import numpy as np
import PIL
import rospy
import sensor_msgs.msg
import torch
import torchvision

from tsm.mobilenet_v2_tsm import MobileNetV2


class GroupResize(object):

    def __init__(self, size, interpolation=PIL.Image.BILINEAR):
        self.worker = torchvision.transforms.Resize(size, interpolation)

    def __call__(self, img_group):
        return [self.worker(img) for img in img_group]


class Stack(object):

    def __init__(self, roll=False):
        self.roll = roll

    def __call__(self, img_group):
        if img_group[0].mode == 'L':
            return np.concatenate(
                [np.expand_dims(x, 2) for x in img_group], axis=2)
        elif img_group[0].mode == 'RGB':
            if self.roll:
                return np.concatenate(
                    [np.array(x)[:, :, ::-1] for x in img_group], axis=2)
            else:
                return np.concatenate(img_group, axis=2)


class ToTorchFormatTensor(object):

    def __init__(self, div=True):
        self.div = div

    def __call__(self, pic):
        if isinstance(pic, np.ndarray):
            # handle numpy array
            img = torch.from_numpy(pic).permute(2, 0, 1).contiguous()
        else:
            # handle PIL Image
            img = torch.ByteTensor(
                torch.ByteStorage.from_buffer(pic.tobytes()))
            img = img.view(pic.size[1], pic.size[0], len(pic.mode))
            # put it from HWC to CHW format
            # yikes, this transpose takes 80% of the loading time/CPU
            img = img.transpose(0, 1).transpose(0, 2).contiguous()
        return img.float().div(255) if self.div else img.float()


def normalize_impl(clip, mean, std, inplace=False):
    if not inplace:
        clip = clip.clone()

    dtype = clip.dtype
    mean = torch.as_tensor(mean, dtype=dtype, device=clip.device)
    std = torch.as_tensor(std, dtype=dtype, device=clip.device)
    clip.sub_(mean[:, None, None]).div_(std[:, None, None])

    return clip


class GroupNormalize(object):

    def __init__(self, mean, std, inplace=False):
        mean = np.array(mean, dtype=np.float32)
        std = np.array(std, dtype=np.float32)
        self.mean = torch.from_numpy(mean).float()
        self.std = torch.from_numpy(std).float()
        self.inplace = inplace

    def __call__(self, clip):
        return normalize_impl(clip, self.mean, self.std, self.inplace)


def get_transform():
    transform = torchvision.transforms.Compose([
        GroupResize(224),
        Stack(roll=False),
        ToTorchFormatTensor(div=True),
        GroupNormalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])
    return transform


class TSMActionRecognitionNode(object):

    def __init__(self):
        super(TSMActionRecognitionNode, self).__init__()
        self.bridge = CvBridge()

        self.class_names = rospy.get_param('~class_names')
        model = MobileNetV2(n_class=len(self.class_names))
        pretrained_model = rospy.get_param('~pretrained_model')
        model.load_state_dict(torch.load(pretrained_model))
        model.eval()
        self.model = model

        self.transform = get_transform()

        self.gpu = rospy.get_param('~gpu', -1)
        if self.gpu >= 0:
            self.device = torch.device("cuda:{}".format(self.gpu))
        else:
            self.device = torch.device('cpu')

        self.temporal_shift_buffer = (
            torch.empty((1, 3, 56, 56),).to(self.device),
            torch.empty((1, 4, 28, 28), ).to(self.device),
            torch.empty((1, 4, 28, 28), ).to(self.device),
            torch.empty((1, 8, 14, 14), ).to(self.device),
            torch.empty((1, 8, 14, 14), ).to(self.device),
            torch.empty((1, 8, 14, 14), ).to(self.device),
            torch.empty((1, 12, 14, 14), ).to(self.device),
            torch.empty((1, 12, 14, 14), ).to(self.device),
            torch.empty((1, 20, 7, 7), ).to(self.device),
            torch.empty((1, 20, 7, 7), ).to(self.device),
        )

        self.model = self.model.to(self.device)

        self.classifier_name = rospy.get_param(
            "~classifier_name", rospy.get_name())

        self.image_pub = rospy.Publisher(
            '~output/viz',
            sensor_msgs.msg.Image,
            queue_size=1)
        self.pub_class = rospy.Publisher(
            "~output/class",
            jsk_recognition_msgs.msg.ClassificationResult,
            queue_size=1)

        self.subscribe()

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 1)
        self.sub_img = rospy.Subscriber(
            '~input', sensor_msgs.msg.Image,
            callback=self.callback,
            queue_size=queue_size)

    def callback(self, img_msg):
        rgb_img = self.bridge.imgmsg_to_cv2(
            img_msg, desired_encoding='rgb8')

        h, w, _ = rgb_img.shape
        if h > w:
            lw = (h - w) // 2
            rw = h - w - lw
            square_img = np.pad(rgb_img, [(0, 0), (lw, rw), (0, 0)],
                                'constant')
        elif w > h:
            lh = (w - h) // 2
            rh = w - h - lh
            square_img = np.pad(rgb_img, [(lh, rh), (0, 0), (0, 0)],
                                'constant')
        else:
            square_img = rgb_img

        img_tran = self.transform(
            [PIL.Image.fromarray(square_img).convert('RGB')])
        input_var = img_tran.view(
            1, 3, img_tran.size(1), img_tran.size(2)).to(self.device)
        with torch.no_grad():
            outputs = self.model(input_var, *self.temporal_shift_buffer)
        features, self.temporal_shift_buffer = outputs[0], outputs[1:]
        features_np = features.detach().cpu().numpy().reshape(-1)
        features_np -= features_np.max()
        scores = np.exp(features_np) / np.sum(np.exp(features_np))

        cls_msg = jsk_recognition_msgs.msg.ClassificationResult(
            header=img_msg.header,
            classifier=self.classifier_name,
            target_names=self.class_names,
            label_names=self.class_names,
            labels=np.arange(len(scores)),
            label_proba=scores,
        )

        self.pub_class.publish(cls_msg)

        if self.image_pub.get_num_connections() > 0:
            height, width, _ = rgb_img.shape
            viz_img = cv2.resize(rgb_img, (width, height))
            viz_img = cv2.cvtColor(viz_img, cv2.COLOR_RGB2BGR)
            label_img = 255 * np.ones((height // 10, width, 3)).astype('uint8')

            idx = np.argmax(scores)
            cv2.putText(label_img, 'Prediction: ' + self.class_names[idx],
                        (0, int(height / 16)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 0), 2)
            viz_img = np.concatenate((viz_img, label_img), axis=0)
            msg = self.bridge.cv2_to_imgmsg(viz_img, "bgr8")
            msg.header = img_msg.header
            self.image_pub.publish(msg)


def main():
    rospy.init_node('tsm_action_recognition_node')
    act = TSMActionRecognitionNode()  # NOQA
    rospy.spin()


if __name__ == '__main__':
    main()
