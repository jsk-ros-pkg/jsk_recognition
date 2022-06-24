#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'jsk_perception'

    download_data(
        pkg_name=PKG,
        path='sample/data/2016-11-11-12-53-06_in_lab.bag',
        url='https://drive.google.com/uc?id=11Oi0nKJpkMQqYg-nftGaWnD3uZ13nFXx',
        md5='5066157f7373a19829f4edba65068a89',
        extract=False,
        compressed_bags=[
            'sample/data/2016-11-11-12-53-06_in_lab.bag',
        ],
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/2016-10-15-23-21-42_moving_bottle.bag',
        url='https://drive.google.com/uc?id=1Puo5mU7Xjv4byGIK-IfvdvsvqPuLe6ii',
        md5='ad4e7d298c0d9985295d93e47c7b03e6',
        extract=False,
        compressed_bags=[
            'sample/data/2016-10-15-23-21-42_moving_bottle.bag',
        ],
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/2017-06-20-12-00-00_people_images_in_lab.bag.tgz',
        url='https://drive.google.com/uc?id=1VYLgjccB9sCa5ht32r3FLjUC2fhFkyZI',
        md5='0a397a2ff14c07954cc0b9178e33600d',
        extract=True,
    )

    # sample/sample_regional_feature_based_object_recognition.launch
    download_data(
        pkg_name=PKG,
        path='sample/data/apc2016_object_imgs_and_masks.bag',
        url='https://drive.google.com/uc?id=1H3PYwHsberifKk8K1zVfKFzEHMhzHB1S',
        md5='fc03e326e0b46f6ecdf9287ddc5f0f68',
    )
    download_data(
        pkg_name=PKG,
        path='sample/data/apc2016_object_imgs_and_masks_templates.tgz',
        url='https://drive.google.com/uc?id=15e-16BiejGM0ANT4S5Mei04MKpTg6ikj',
        md5='df30d3bf32fda521f4c888c83a394fe5',
        extract=True,
    )
    download_data(
        pkg_name=PKG,
        path='sample/data/resnet_features_apc2016.npz',
        url='https://drive.google.com/uc?id=1cbSIrgBpllXzIq52K5Vw7mgI22Ag2eXA',
        md5='ff7f6a90090d152e2c43bf5057d4e853',
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/2019-07-18-15-37-50_pr2_self_see.bag',
        url='https://drive.google.com/uc?id=1MvSfxqOnRjQeaCM4QF9Fs_Ye9HZKFXwa',
        md5='a98d10a058a74458f41cc7b1be5acea8',
        extract=False,
    )


    # d435 floor
    download_data(
        pkg_name=PKG,
        path='sample/data/2020-11-11-15-58-47-d435-floor.bag',
        url='https://drive.google.com/uc?id=1B1oC6C9ZrjOQTScmbWwR_Y91GAFIg4bu',
        md5='6b07e3c8a4b5a012bcddaa63ca2b4bdf',
        extract=False,
        compressed_bags=[
            'sample/data/2020-11-11-15-58-47-d435-floor.bag',
        ],
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
        path='sample/data/2022-04-07-18-36-43_hand_pose_estimation_2d.bag',
        url='https://drive.google.com/uc?id=1cyXdDPrdOiee9xOE8fW-3TWDCbCuwLBP',
        md5='5ab7a29cba0f499b684e767458795be1',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/sample_rect_array_in_panorama_to_bounding_box_array.bag',
        url='https://drive.google.com/uc?id=1f613TsYuPk1DWuGuUfkJVBbPv2-4Io69',
        md5='04ba99ee993860634c0064d167b5cbe5',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/sample_mask_rcnn_73b2_kitchen.bag',
        url='https://drive.google.com/uc?id=1DZ6W7gBzbu-7Z-NRw7y0WCGDRpsaYadI',
        md5='f86a51988fe521863ff845a2c9ff10df',
        extract=False,
        compressed_bags=[
            'sample/data/sample_mask_rcnn_73b2_kitchen.bag',
        ],
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/sample_mask_rcnn_73b2_kitchen_japanese_label.bag',
        url='https://drive.google.com/uc?id=1Kfgj7rkuQB14o73t7FbkAee19oNKkYSt',
        md5='83c23c1005794d4ee5b7461c23f3b259',
        extract=False,
        compressed_bags=[
            'sample/data/sample_mask_rcnn_73b2_kitchen_japanese_label.bag',
        ],
    )
    # lidar person detection
    download_data(
        pkg_name=PKG,
        path='sample/data/2022-06-24-20-26-33-people_lidar_in_lab.bag',
        url='https://drive.google.com/uc?id=16MRv5YqPm_pzlSMkn0C3nWQ9txaglBMw',
        md5='36a1951e90c550cc537f4e07004d87e8',
        extract=False,
    )


if __name__ == '__main__':
    main()
