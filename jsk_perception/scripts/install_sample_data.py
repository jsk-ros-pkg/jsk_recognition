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

    download_data(
        pkg_name=PKG,
        path='sample/data/insta360_air.bag.tgz',
        url='https://drive.google.com/uc?id=1f99gIK92EsGvVI5ahHAV5g8xoEHyQhtv',
        md5='762eb0591f0ab03718733596c8d01728',
        extract=True,
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/fisheye_stitcher_grid_xd_yd_3840x1920_fetch15.yml.gz',
        url='https://drive.google.com/uc?id=1WKnd-gDgp3GCWOAPtwg1lkrPqBiIyB_l',
        md5='edf9b561446db316925a110841e9a918',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/fisheye_stitcher_grid_xd_yd_3840x1920_fetch1075.yml.gz',
        url='https://drive.google.com/uc?id=1SZwyUT14uEDuKvBzv6K0iOcYJoIIbaI0',
        md5='762f88556a8e43551aef261e25e2468b',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/fisheye_stitcher_grid_xd_yd_3840x1920_oem.yml.gz',
        url='https://drive.google.com/uc?id=1VFK46da5CMIYaUaDw5DVy2ZQOTrOn-dO',
        md5='2c16173cb823a1d77c14a2272e224e00',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/sample_rect_array_in_panorama_to_bounding_box_array.bag',
        url='https://drive.google.com/uc?id=1f613TsYuPk1DWuGuUfkJVBbPv2-4Io69',
        md5='04ba99ee993860634c0064d167b5cbe5',
        extract=False,
    )


if __name__ == '__main__':
    main()
