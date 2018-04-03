#!/usr/bin/env python

import argparse
from distutils.version import LooseVersion
import multiprocessing
import os.path as osp
import sys

try:
    import chainer
except:
    print('### Failed to import chainer')

import jsk_data


def download_data(*args, **kwargs):
    p = multiprocessing.Process(
            target=jsk_data.download_data,
            args=args,
            kwargs=kwargs)
    p.start()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    quiet = args.quiet

    PKG = 'jsk_perception'

    download_data(
        pkg_name=PKG,
        path='trained_data/drill_svm.xml',
        url='https://drive.google.com/uc?id=0B5hRAGKTOm_KWW11R0FTX0xjTDg',
        md5='762d0da4bcbf50e0e92939372988901a',
        quiet=quiet,
    )

    download_data(
        pkg_name=PKG,
        path='trained_data/apc2015_sample_bof.pkl.gz',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vemVRaDBOWDVpb28',
        md5='97eb737f71a33bfc23ec573f1d351bd8',
        quiet=quiet,
    )
    download_data(
        pkg_name=PKG,
        path='trained_data/apc2015_sample_clf.pkl.gz',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2veFY5ZFNqbzAzNmc',
        md5='25e396358e9d7bfd1bd08334953fc287',
        quiet=quiet,
    )

    files = [
        ('ObjNessB2W8HSV.idx.yml.gz', 'e066c100d60246a3911d4559182d9d2a'),
        ('ObjNessB2W8HSV.wS1.yml.gz', '728507d99521d7dba9b0eb114ccbb830'),
        ('ObjNessB2W8HSV.wS2.yml.gz', '790e27251267d86168a12f2bd2d96f8d'),
        ('ObjNessB2W8I.idx.yml.gz', '9425dd4d31521fced82aeb6fc56ce4d5'),
        ('ObjNessB2W8I.wS1.yml.gz', 'a04d4b4504887fc16800b8b42bac9e70'),
        ('ObjNessB2W8I.wS2.yml.gz', 'f2e2f5726e352bfa16224066e2bdc7ad'),
        ('ObjNessB2W8MAXBGR.idx.yml.gz', 'ef2fbd5da0ffb5fe4332685b8529dc5c'),
        ('ObjNessB2W8MAXBGR.wS1.yml.gz', 'cbe8147fca9a5885b7bb25d38fa5f4d1'),
        ('ObjNessB2W8MAXBGR.wS2.yml.gz', '02b76364df35cef862da041585b537de'),
    ]
    dirname = 'https://github.com/Itseez/opencv_contrib/raw/3.1.0/modules/saliency/samples/ObjectnessTrainedModel'  # NOQA
    for fname, md5 in files:
        download_data(
            pkg_name=PKG,
            path=osp.join('trained_data/ObjectnessTrainedModel/', fname),
            url=osp.join(dirname, fname),
            md5=md5,
            quiet=quiet,
        )

    # node_scripts/fast_rcnn.py
    download_data(
        pkg_name=PKG,
        path='trained_data/vgg16_fast_rcnn.chainermodel',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vX015UzB4aC13cVk',
        md5='5ae12288962e96124cce212fd3f18cad',
        quiet=quiet,
    )
    download_data(
        pkg_name=PKG,
        path='trained_data/vgg_cnn_m_1024.chainermodel',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vZzJuaFRIdDMtLWc',
        md5='eb33103e36f299b4433c63fcfc165cbd',
        quiet=quiet,
    )
    download_data(
        pkg_name=PKG,
        path='trained_data/vgg16_bn_apc2015_496000.chainermodel',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vQ2tCN1hoYV84eHM',
        md5='4a48c2f39234e46937759f4cc43bb257',
        quiet=quiet,
    )

    # node_scripts/fcn_object_segmentation.py
    # ref: https://github.com/wkentaro/fcn#training
    download_data(
        pkg_name=PKG,
        path='trained_data/fcn8s_voc.npz',
        url = 'https://drive.google.com/uc?id=0B9P1L--7Wd2vWG5MeUEwWmxudU0',
        md5 = '75128c0e175767fc82a7d4f1e21f4009',
    )

    # node_scripts/vgg16_object_recognition.py
    download_data(
        pkg_name=PKG,
        path='trained_data/bvlc_vgg16.chainermodel',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vSlFjQlJFQjM5TEk',
        md5='292e6472062392f5de02ef431bba4a48',
    )

    # node_scripts/alexnet_object_recognition.py
    download_data(
        pkg_name=PKG,
        path='trained_data/bvlc_alexnet.chainermodel',
        url='https://drive.google.com/uc?id=0B5DV6gwLHtyJZkd1ZTRiNUdrUXM',
        md5='2175620a2237bbd33e35bf38867d84b2',
    )

    # node_scripts/people_pose_estimation_2d.py
    path = 'trained_data/pose_estimation_2d_chainermodel.pkl'
    if not 'chainer' in sys.modules or LooseVersion(chainer.__version__) >= LooseVersion('2.0.0'):
        # created on chainer v2.0.0
        download_data(
            pkg_name=PKG,
            path=path,
            url='https://drive.google.com/uc?id=0B_NiLAzvehC9R2stRmQyM3ZiVjQ',
            md5='587933c2c0adf335ebed0486c183541f',
        )
    else:
        # created on chainer v1.24.0
        download_data(
            pkg_name=PKG,
            path=path,
            url='https://drive.google.com/uc?id=0B4ysRIwB7GryNnhidGN3VVJkNVE',
            md5='4d41e1ac80185849384a67a329746115',
        )

    # node_scripts/feature_based_object_recognition.py
    download_data(
        pkg_name=PKG,
        path='trained_data/resnet_lsvrc2012_mean.npy',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vTDV3ZzUyTlBFZE0',
        md5='00431426c4fab22985885da0e2ff31b8',
    )
    download_data(
        pkg_name=PKG,
        path='trained_data/resnet152_from_caffe.npz',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vQVBodlFsMnpGbkU',
        md5='77fe66a229a2444688a21e3b63fa0661',
    )

    # node_scripts/fcn_depth_prediction.py
    download_data(
        pkg_name=PKG,
        path='trained_data/fcn8s_depth_prediction_refrigerator.npz',
        url='https://drive.google.com/uc?id=15n00783FVwxrG9DRdBQOmi8xu1pz-FYl',
        md5='a585e4d41ed67d5052417ade6fb2d608',
    )

    # node_scripts/mask_rcnn_instance_segmentation.py
    download_data(
        pkg_name=PKG,
        path='trained_data/mask_rcnn_resnet50_voc_20180403',
        url='https://drive.google.com/uc?id=1Ui_SXZzF388fLZ-WM1rponFziCCJcm3n',
        md5='16a196fab8789f7c72a71241a85f59f7',
    )


if __name__ == '__main__':
    main()
