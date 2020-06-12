#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'jsk_perception'

    download_data(
        pkg_name=PKG,
        path='sample/data/2016-11-11-12-53-06_in_lab.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vdE9IRERDLXRoUWs',
        md5='5066157f7373a19829f4edba65068a89',
        extract=False,
        compressed_bags=[
            'sample/data/2016-11-11-12-53-06_in_lab.bag',
        ],
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/2016-10-15-23-21-42_moving_bottle.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vTDZOZGhrTTNvc00',
        md5='ad4e7d298c0d9985295d93e47c7b03e6',
        extract=False,
        compressed_bags=[
            'sample/data/2016-10-15-23-21-42_moving_bottle.bag',
        ],
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/2017-06-20-12-00-00_people_images_in_lab.bag.tgz',
        url='https://drive.google.com/uc?id=0B_NiLAzvehC9Yzl4YzB1WnlFcXM',
        md5='0a397a2ff14c07954cc0b9178e33600d',
        extract=True,
    )

    # sample/sample_regional_feature_based_object_recognition.launch
    download_data(
        pkg_name=PKG,
        path='sample/data/apc2016_object_imgs_and_masks.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2veVJ4N21tVlB3MGs',
        md5='fc03e326e0b46f6ecdf9287ddc5f0f68',
    )
    download_data(
        pkg_name=PKG,
        path='sample/data/apc2016_object_imgs_and_masks_templates.tgz',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vX0NqcXNGYXEzUlE',
        md5='df30d3bf32fda521f4c888c83a394fe5',
        extract=True,
    )
    download_data(
        pkg_name=PKG,
        path='sample/data/resnet_features_apc2016.npz',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vM3hhOTBGNjA1eTQ',
        md5='ff7f6a90090d152e2c43bf5057d4e853',
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/2019-07-18-15-37-50_pr2_self_see.bag',
        url='https://drive.google.com/uc?id=1MvSfxqOnRjQeaCM4QF9Fs_Ye9HZKFXwa',
        md5='a98d10a058a74458f41cc7b1be5acea8',
        extract=False,
    )


if __name__ == '__main__':
    main()
