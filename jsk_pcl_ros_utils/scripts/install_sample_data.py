#!/usr/bin/env python

import argparse
import os.path as osp

import jsk_data


def download_data(path, url, md5, pkg_path=None):
    if pkg_path is not None:
        path = osp.join(pkg_path, path)
    return jsk_data.download_data(
        pkg_name='jsk_pcl_ros_utils',
        path=path,
        url=url,
        md5=md5,
    )


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        '--pkg-path', help='PAKCAGE_SOURCE_DIR in cmake'
    )
    args = parser.parse_args()

    download_data(
        pkg_path=args.pkg_path,
        path='sample/data/2017-02-05-16-11-09_shelf_bin.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vYWI2NnZrekEwSmc',
        md5='44427634f57ac76111edabd7b1f4e140',
    )

    download_data(
        pkg_path=args.pkg_path,
        path='sample/data/bunny_marker_array.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vdW1NMlhiRU9KZDQ',
        md5='e7dc29d21bdd30c769396c361e4350fd',
    )

    download_data(
        pkg_path=args.pkg_path,
        path='sample/data/bunny.pcd',
        url='https://raw.githubusercontent.com/PointCloudLibrary/pcl/pcl-1.8.0/test/bunny.pcd',  # NOQA
        md5='a4e58778ba12d3f26304127f6be82897',
    )

    download_data(
        pkg_path=args.pkg_path,
        path='sample/data/arc2017_4objects.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vakpvU0wtMFNCTkk',
        md5='2c3af4482cd2e0ee95b58848ae48afaf',
    )


if __name__ == '__main__':
    main()
