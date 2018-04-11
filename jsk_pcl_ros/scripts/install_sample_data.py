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
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vUllPT3VPOHN6LVU',
        md5='dbc3daff4fe9812cab7328e7702e6438',
        extract=True,
    )

    download_data(
        path='sample/data/2017-06-19-19-34-18_tabletop_coffeecup.tgz',
        url='https://drive.google.com/uc?id=0B5DV6gwLHtyJRTJ0eGdVYk43bEU',
        md5='d58e894c853fc4485d08547b9fc2b640',
        extract=True,
    )

    download_data(
        path='sample/data/2016-10-26-02-09-51_coffee_cup.pcd',
        url='https://drive.google.com/uc?id=0B5DV6gwLHtyJVVQ2TFhVSTJqZ3M',
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
        url='https://drive.google.com/uc?id=0B_NiLAzvehC9cHhnOFd0YUN2N1U',
        md5='c59067adc429fb9f1cf180d350a2da43',
        extract=True,
        compressed_bags=[
            'sample/data/2016-07-06-12-16-43-person-in-point-cloud/vision.compressed.bag',  # NOQA
        ],
    )

    download_data(
        path='sample/data/stereo_arc2017_objects.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2va1ZvWHJRT3Y5dkk',
        md5='4d8f51d0827f66afe5b91473aeaf9c97',
    )

    download_data(
        path='sample/data/room73b2_table.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vdXdfMWNSUU9IVEU',
        md5='a38fee55c926152f3d65023d60322eb0',
    )

    download_data(
        path='sample/data/sample_add_color_from_image_20170319.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vN284RHBXT1duUWM',
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
        url='https://drive.google.com/uc?id=0B4ysRIwB7GryMGxCTUI0MHhqWFk',
        md5='f702e012730db7aaf01b9868280d8bca',
        compressed_bags=[
            'sample/data/sample_door_handle_detector.bag',
        ],
    )


if __name__ == '__main__':
    main()
