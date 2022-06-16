#!/usr/bin/env python
# -*- coding: utf-8 -*-

# create dataset for training with chainer.
# some data augmentation is executed for training data (not for test data)

# directory composition
# original_spectrogram - classA -- 001.png
#                      |        |- 002.png
#                      |        |- ...
#                      - classB -- 001.png
#                               |- 002.png
#                               |- ...
#
# -> (./create_dataset.py)
#
# original_spectrogram - classA -- 001.png
#                      |        |- 002.png
#                      |        |- ...
#                      - classB -- 001.png
#                               |- 002.png
#                               |- ...
#
# dataset -- train_images.txt  # necessary for chainer
#         |- test_images.txt   # necessary for chainer
#         |- train_(class)000*.png
#         |- ...
#         |- test_(class)000*.png
#         |- ...
#
# n_class.txt
#
# Total data number
# train: (number of images per class) * (train:test rate)       * (augment number)
# test : (number of images per class) * (1 - (train:test rate))

import argparse
from sound_classification.process_gray_image import img_jet
import imgaug as ia
import imgaug.augmenters as iaa
from PIL import Image as Image_
import numpy as np
import os
import os.path as osp
import random
import rospkg
import shutil


# for data augmentation
ia.seed(1)
st = lambda x: iaa.Sometimes(0.3, x)
seq = iaa.Sequential([
    st(iaa.GaussianBlur(sigma=(0, 0.5))),
    st(iaa.ContrastNormalization((0.75, 1.5))),
    st(iaa.AdditiveGaussianNoise(loc=0, scale=(0.0, 0.05*255), per_channel=0.5)),
    st(iaa.Multiply((0.8, 1.2), per_channel=0.2)),
    st(iaa.Affine(
        scale={"x": (0.8, 1.2), "y": (1.0, 1.0)},
        translate_percent={"x": (-0.2, 0.2), "y": (0, 0)},
    ))
], random_order=True)  # apply augmenters in random order

rospack = rospkg.RosPack()


# Split dataset into train data and test data. The rate is given by --rate.
def split():
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--rate', default='0.8', type=float,
                        help='train:test dataset rate (default 0.8:0.2)')
    parser.add_argument('-p', '--path', default=osp.join(rospack.get_path(
        'sound_classification'), 'train_data'), help='path to train data')
    parser.add_argument('-a', '--augment', default='1', type=int,
                        help='create {augment} images per 1 image')
    parser.add_argument('-m', '--model', type=str,
                        choices=['nin', 'vgg16'], default='nin',
                        help='Neural network model to use dataset')
    parser.add_argument('-n', '--number', default='100', type=int,
                        help='maximum number of images per class used to create dataset')
    # Ignore arguments sent by roslaunch.
    parser.add_argument('__name:', help=argparse.SUPPRESS, nargs='?')
    parser.add_argument('__log:', help=argparse.SUPPRESS, nargs='?')

    args = parser.parse_args()
    rate = args.rate
    if args.model == 'nin':
        image_size = (227, 227)
    elif args.model == 'vgg16':
        image_size = (224, 224)
    else:
        print('Model type {} is invalid.'.format(args.model))
        exit()
    root_dir = osp.expanduser(args.path)
    origin_dir = osp.join(root_dir, 'original_spectrogram')
    dataset_dir = osp.join(root_dir, 'dataset')
    image_list_train = []
    image_list_test = []
    mean_of_dataset = np.zeros((image_size[0], image_size[1], 3)).astype(np.float32)
    size_of_dataset = 0

    if osp.exists(dataset_dir):
        shutil.rmtree(dataset_dir)
    os.mkdir(dataset_dir)
    # write how many classes
    classes = sorted(os.listdir(origin_dir))
    with open(osp.join(root_dir, 'n_class.txt'), mode='w') as f:
        for class_name in classes:
            f.write(class_name + '\n')
    for class_id, class_name in enumerate(classes):
        file_names = os.listdir(osp.join(origin_dir, class_name))
        file_num = len(file_names)
        # copy train and test data
        # resize and augment data (multiple args.augment times)
        image_num_per_class = min(args.number, file_num)
        selected_images = random.sample(list(range(file_num)), image_num_per_class)
        for i, file_name in enumerate(np.array(file_names)[selected_images]):
            if file_name.endswith('.png') is not True:
                continue
            saved_file_name = class_name + '_' + file_name
            img = Image_.open(osp.join(origin_dir, class_name, file_name))
            img = img_jet(np.asarray(img))[:, :, [2, 1, 0]]  # bgr -> rgb
            img = Image_.fromarray(img)
            img_resize = img.resize((image_size[0], image_size[1]))
            mean_of_dataset += img_resize
            size_of_dataset += 1
            if i < image_num_per_class * rate:  # save data for train
                saved_file_name = 'train_' + saved_file_name
                for j in range(args.augment):
                    _ = osp.splitext(saved_file_name)
                    saved_file_name_augmented = _[0] + '_{0:03d}'.format(j) + _[1]
                    img_aug = Image_.fromarray(seq.augment_image(np.array(img_resize)))
                    img_aug.save(osp.join(dataset_dir, saved_file_name_augmented))
                    image_list_train.append(saved_file_name_augmented + ' ' + str(class_id) + '\n')
                    print('saved {}'.format(saved_file_name_augmented))
            else:  # save data for test
                saved_file_name = 'test_' + saved_file_name
                img_resize.save(osp.join(dataset_dir, saved_file_name))
                image_list_test.append(saved_file_name + ' ' + str(class_id) + '\n')
                print('saved {}'.format(saved_file_name))

        # create images.txt
        # for train
        file_path = osp.join(dataset_dir, 'train_images.txt')
        with open(file_path, mode='w') as f:
            for line_ in image_list_train:
                f.write(line_)
        # for test
        file_path = osp.join(dataset_dir, 'test_images.txt')
        with open(file_path, mode='w') as f:
            for line_ in image_list_test:
                f.write(line_)

    # save mean value of dataset
    file_path = osp.join(dataset_dir, 'mean_of_dataset.png')
    saved_image = Image_.fromarray(
        (mean_of_dataset / size_of_dataset).astype(np.uint8))
    saved_image.save(file_path)


if __name__ == '__main__':
    split()
