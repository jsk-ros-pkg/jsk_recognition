#!/usr/bin/env python

import argparse
import multiprocessing
import os.path as osp

try:
    import chainer  # NOQA
    _chainer_available = True
except:
    print('### Failed to import chainer')
    _chainer_available = False

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
        path='trained_data/apc2015_sample_bof_sklearn==0.20.0.pkl.gz',
        url='https://drive.google.com/uc?id=1VRwQxbjtSI4I1cjIqUFaemiUTHE4wlDj',
        md5='001dbd0767369daff0cafb8fc7b39e92',
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
    if _chainer_available:
        download_data(
            pkg_name=PKG,
            path=path,
            url='https://drive.google.com/'
            'uc?id=1la-B-I1Dh00BRkJuNC3TAXju6p3ccmmb',
            md5='c0683094aa42eab1b9424e05112190c5',
        )

    path = 'trained_data/hand_ssd300_chainermodel.npz'
    if _chainer_available:
        download_data(
            pkg_name=PKG,
            path=path,
            url='https://drive.google.com/'
            'uc?id=1rJ_ZYY-AjKqvlJGLF6RJ_I_vuaOp3lXg',
            md5='ba1226f8dd816514e610a746278be02e',
        )
    # node_scripts/human_mesh_recovery.py
    if _chainer_available:
        download_data(
            pkg_name=PKG,
            path='trained_data/hmr_smpl.npz',
            url='https://drive.google.com/'
            'uc?id=10TIlcXBdKreTapQuZEIjWeeWwxG32gM6',
            md5='d4a0c097b0ee26b93fa07f83c1c5e259',
        )
        download_data(
            pkg_name=PKG,
            path='trained_data/hmr_resnet_v2_50_model.npz',
            url='https://drive.google.com/'
            'uc?id=1_JGxDnANk1pj23PW3T4JFRfei6Qs2Wwz',
            md5='742a129d5b6dd62e71a081973128beb9',
        )
        download_data(
            pkg_name=PKG,
            path='trained_data/hmr_encoder_fc3_model.npz',
            url='https://drive.google.com/'
            'uc?id=19nGjVyIaXMhILS32J4whQgApY_qKYURj',
            md5='33d80575b507b66c975f350f2f24ee91',
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
        path='trained_data/mask_rcnn_resnet50_voc_20180516.npz',
        url='https://drive.google.com/uc?id=1uv_jK-CAIJUXsRNmccFEISSKW4vXqI46',
        md5='47a507934b6bc20f0d9274825b734942',
    )


if __name__ == '__main__':
    main()
