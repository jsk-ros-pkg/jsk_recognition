#!/usr/bin/env python

from __future__ import print_function

import argparse
import glob
import os.path as osp

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
import chainer
from chainer import cuda
import numpy as np
import rospkg
import skimage.io
from sklearn.metrics import classification_report
from sklearn.neighbors import KNeighborsClassifier
import tqdm
import yaml

# TODO(wkentaro): Support Resnet50/101
from jsk_recognition_utils.chainermodels import ResNet152Feature


def get_templates(template_dir):
    target_names = yaml.load(open(osp.join(template_dir, 'target_names.yaml')))
    for cls_id, cls_name in enumerate(target_names):
        obj_dir = osp.join(template_dir, cls_name)
        for img_file in glob.glob(osp.join(obj_dir, '*.jpg')):
            dirname, basename = osp.split(img_file)
            mask_file = osp.join(dirname, 'masks', basename)
            img = skimage.io.imread(img_file)
            mask = skimage.io.imread(mask_file) >= 127
            yield cls_id, img, mask


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('template_dir', help='Template dir')
    parser.add_argument('db_file', help='DB file which will be created')
    parser.add_argument('-g', '--gpu', type=int, default=0)
    args = parser.parse_args()

    template_dir = args.template_dir
    db_file = args.db_file
    gpu = args.gpu

    pkg_path = rospkg.RosPack().get_path('jsk_perception')
    mean_file = osp.join(pkg_path, 'trained_data/resnet_lsvrc2012_mean.npy')
    pretrained_model = osp.join(
        pkg_path, 'trained_data/resnet152_from_caffe.npz')

    target_names = yaml.load(open(osp.join(template_dir, 'target_names.yaml')))

    mean = np.load(mean_file)

    model = ResNet152Feature()
    chainer.serializers.load_npz(pretrained_model, model)
    if gpu >= 0:
        chainer.cuda.get_device_from_id(gpu).use()
        model.to_gpu()

    chainer.global_config.train = False
    chainer.global_config.enable_backprop = False

    ###########################################################################

    X = []
    y = []
    for cls_id, img, mask in tqdm.tqdm(get_templates(template_dir)):
        img = img[:, :, ::-1]  # RGB- > BGR
        img = img.astype(np.float64)
        img[mask] -= mean[mask]
        img[~mask] = 0

        img = img.transpose(2, 0, 1)
        img = img.astype(np.float32)
        x_data = np.asarray([img])
        x_data = cuda.to_gpu(x_data)
        x = chainer.Variable(x_data)
        feat = model(x)

        feat = cuda.to_cpu(feat.data)
        feat = feat.squeeze(axis=(2, 3))
        for f in feat:
            X.append(f)
            y.append(cls_id)
    X = np.asarray(X)
    y = np.asarray(y)
    np.savez_compressed(db_file, X=X, y=y, target_names=target_names)

    knn = KNeighborsClassifier(n_neighbors=1)
    knn.fit(X, y)
    y_pred = knn.predict(X)
    # validation: must be all 1.0
    print(classification_report(y, y_pred, labels=range(len(target_names)),
                                target_names=target_names))


if __name__ == '__main__':
    main()
