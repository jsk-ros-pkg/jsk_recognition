#!/usr/bin/env python

import argparse
import multiprocessing

import jsk_data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    quiet = args.quiet

    def download_data(**kwargs):
        kwargs['pkg_name'] = 'jsk_pcl_ros'
        kwargs['quiet'] = quiet
        p = multiprocessing.Process(
            target=jsk_data.download_data,
            kwargs=kwargs)
        p.start()

    download_data(
        path='sample/data/2016-06-24-17-43-57_tabletop.bag.tgz',
        url='https://drive.google.com/uc?id=1aIOrcLu7OxpkEGdUBCC6FWGb6NjP_dkn',
        md5='dbc3daff4fe9812cab7328e7702e6438',
        extract=True,
    )

    download_data(
        path='sample/data/2017-06-19-19-34-18_tabletop_coffeecup.tgz',
        url='https://drive.google.com/uc?id=16uMAe5y5fuqSJpCk_iH6gBBHsyI4Llob',
        md5='58002c60761d192322413f799368ae88',
        extract=True,
    )

    download_data(
        path='sample/data/2016-10-26-02-09-51_coffee_cup.pcd',
        url='https://drive.google.com/uc?id=1YiO4UpRCfpskzUvCFmMyaY8mqaKavGlx',
        md5='27d1a39e6ea596c4e24f1347f53f2e7b',
        extract=False,
    )

    download_data(
        path='sample/data/drill.pcd',
        url='http://www.jsk.t.u-tokyo.ac.jp/~ueda/dataset/2015/02/drill.pcd',
        md5='b6ea8f7bd97fd1e88fb6af2f6cd42ac5',
        extract=False,
    )

    download_data(
        path='sample/data/2016-07-06-12-16-43-person-in-point-cloud.tgz',
        url='https://drive.google.com/uc?id=10cN9sfxOPWqieqIBb6dKs3yJMsG0tH4n',
        md5='c59067adc429fb9f1cf180d350a2da43',
        extract=True,
        compressed_bags=[
            'sample/data/2016-07-06-12-16-43-person-in-point-cloud/vision.compressed.bag',  # NOQA
        ],
    )

    download_data(
        path='sample/data/stereo_arc2017_objects.bag',
        url='https://drive.google.com/uc?id=1TP0h-uf3A_f9SeIiUiwApDiKII3yA-9E',
        md5='4d8f51d0827f66afe5b91473aeaf9c97',
    )

    download_data(
        path='sample/data/room73b2_table.bag',
        url='https://drive.google.com/uc?id=1qKWPVGxLXaaKAeIXM-XhK7WP05I1X3-s',
        md5='a38fee55c926152f3d65023d60322eb0',
    )

    download_data(
        path='sample/data/sample_add_color_from_image_20170319.bag',
        url='https://drive.google.com/uc?id=1eZZmW4uvlkNMUzDkztrs1gtd91Y3f4QE',
        md5='d432e7eec587708849ac85fdb3c6247f',
        compressed_bags=[
            'sample/data/sample_add_color_from_image_20170319.bag',
        ],
    )

    download_data(
        path='sample/data/pr2_sink.bag.tgz',
        url='https://drive.google.com/uc?id=19VujodKsX7EJJwHetqFGCRYz0JBtZQ6h',
        md5='9bdc1500610a1a97480cae6fcf261811',
        extract=True,
        compressed_bags=[
            'sample/data/pr2_sink.bag',
        ],
    )

    download_data(
        path='sample/data/kettle.pcd',
        url='https://drive.google.com/uc?id=1MpQ3MK3D5VTlmj--r9dFPrfyLPQU7LO0',
        md5='87579bdbb5f87a058697ace16887e37f',
        extract=False,
    )

    download_data(
        path='sample/data/sample_door_handle_detector.bag',
        url='https://drive.google.com/uc?id=1CTP_l4NHy7YSiPKPGt6ek-xnHOCMd9IA',
        md5='f702e012730db7aaf01b9868280d8bca',
        compressed_bags=[
            'sample/data/sample_door_handle_detector.bag',
        ],
    )

    download_data(
        path='sample/data/octomap_contact.bag.tgz',
        url='https://drive.google.com/uc?id=1r7Hj4ujB8Ml5QHg7_Q6kd3rbtL6kzTbh',
        md5='a6f4d560830c93e0710fef0e7518a099',
        extract=True,
        compressed_bags=[
            'sample/data/octomap_contact.bag',
        ],
    )

    download_data(
        path='sample/data/pr2_look_around_in_room_73b2.bag',
        url='https://drive.google.com/uc?id=1gmEUanixU-3Fvsff-3e8LkXmZECJCcE2',
        md5='6e3dfda0ac4f23b7849f135ec9041e92',
    )

    download_data(
        path='sample/data/convenience_store.bag',
        url='https://drive.google.com/uc?id=1OyhDsoPffgX0pHiF4XfXI3wM5FK3USKY',
        md5='d7bc0f2e9b126c013c18964d4a1fa1be',
    )

    download_data(
        path='sample/data/torus.pcd',
        url='https://drive.google.com/uc?id=18VRHlcbHNFw6Hi5ejLPMDI2iqaEdlpTU',
        md5='17ae9636f42403a20bf1b936c03bb35c',
        extract=False,
    )
    download_data(
        path='sample/data/plane_extraction.bag',
        url='https://drive.google.com/uc?id=1doj3XMY0KYVAkIubmsi5-Oec4y0vvVP2',
        md5='dddd2c6d7738ac19536a032aed8f9967',
    )

    download_data(
        path='sample/data/baxter_realsense_l515.bag',
        url='https://drive.google.com/uc?id=1RKZLA4J2_lzhCtvMTWxGQh4ykVVEeSAd',
        md5='2bb80d7c31c62c944192b2dd0a75eddf',
    )

    download_data(
        path='sample/data/fetch_trashbin_top.bag',
        url='https://drive.google.com/uc?id=1amwEWa2ZYL4Tbgxu0nkBQ_xfW0gryDLI',
        md5='90e9f794ac99ac05eb0dfd74eade3d40',
    )


if __name__ == '__main__':
    main()
