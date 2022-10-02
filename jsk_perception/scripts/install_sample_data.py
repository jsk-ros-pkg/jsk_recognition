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

    download_data(
        pkg_name=PKG,
        path='sample/data/sample_panorama_camera_00.bag',
        url='https://drive.google.com/uc?id=1mEbYBPQ_cQ4arRx3F0mZCKYvd2ptimPK',
        md5='04ba99ee993860634c0064d167b5cbe5',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/sample_panorama_camera_01.bag',
        url='https://drive.google.com/uc?id=1opSX6Mz27CT29PaDKoRvXRW4ewAdk-rd',
        md5='0083c29afae51e6f8f53f4e7f3f89d46',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/sample_panorama_camera_02.bag',
        url='https://drive.google.com/uc?id=1OZyNjMD4UtyBOqfOL74Kn7_yfuG3g1q0',
        md5='08eab7f8acdbd23e6ed48740fd1cd675',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/sample_panorama_camera_03.bag',
        url='https://drive.google.com/uc?id=11DVzyFKF_fRwWGb7-UdaEivLbEfI8LNO',
        md5='ca5eb04f489b093129f9575e8f24c0fc',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/sample_panorama_camera_04.bag',
        url='https://drive.google.com/uc?id=1Iwom5IMfcQo5ggWohvcBW-q1j0uVY_XA',
        md5='c8a7720f5d1aee591726697db9f05a8c',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/sample_panorama_camera_05.bag',
        url='https://drive.google.com/uc?id=1HrsCVpvrOxfKG9_m77rUzNMpHb-Hl5zx',
        md5='57620476da225f3fd7302dfb30c4eb68',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/sample_panorama_camera_06.bag',
        url='https://drive.google.com/uc?id=1z7jXYYO4bOfF_KnddnkW8eXMk07onXx_',
        md5='3cf258ef788ffb937ad926aa5281cdb3',
        extract=False,
    )


if __name__ == '__main__':
    main()
