#!/usr/bin/env python

import argparse
import multiprocessing
import os.path as osp

import jsk_data


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        '--pkg-path', help='PAKCAGE_SOURCE_DIR in cmake'
    )
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    quiet = args.quiet
    pkg_path = args.pkg_path

    def download_data(**kwargs):
        path = kwargs.pop('path')
        if pkg_path is not None:
            path = osp.join(pkg_path, path)
        kwargs['path'] = path
        kwargs['pkg_name'] = 'jsk_pcl_ros_utils'
        kwargs['quiet'] = quiet
        p = multiprocessing.Process(
            target=jsk_data.download_data,
            kwargs=kwargs)
        p.start()

    download_data(
        path='sample/data/2017-02-05-16-11-09_shelf_bin.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vYWI2NnZrekEwSmc',
        md5='44427634f57ac76111edabd7b1f4e140',
    )

    download_data(
        path='sample/data/2019-11-03-hsr-multi-objects-in-shelf.bag',
        url='https://drive.google.com/uc?id=1RvpUWnPJogYw_n7IRfmt6ONpUD1ZETus',
        md5='f719f3dfed7d9cece1557f340f088683',
        extract=False,
        compressed_bags=[
            'sample/data/2019-11-03-hsr-multi-objects-in-shelf.bag',  # NOQA
        ],
    )

    download_data(
        path='sample/data/bunny_marker_array.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vdW1NMlhiRU9KZDQ',
        md5='e7dc29d21bdd30c769396c361e4350fd',
    )

    download_data(
        path='sample/data/bunny.pcd',
        url='https://raw.githubusercontent.com/PointCloudLibrary/pcl/pcl-1.8.0/test/bunny.pcd',  # NOQA
        md5='a4e58778ba12d3f26304127f6be82897',
    )

    download_data(
        path='sample/data/arc2017_4objects.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vakpvU0wtMFNCTkk',
        md5='2c3af4482cd2e0ee95b58848ae48afaf',
    )


if __name__ == '__main__':
    main()
