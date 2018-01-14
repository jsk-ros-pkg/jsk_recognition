#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'jsk_pcl_ros_utils'

    download_data(
        pkg_name=PKG,
        path='sample/data/2017-02-05-16-11-09_shelf_bin.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vYWI2NnZrekEwSmc',
        md5='44427634f57ac76111edabd7b1f4e140',
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/bunny_marker_array.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vdW1NMlhiRU9KZDQ',
        md5='e7dc29d21bdd30c769396c361e4350fd',
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/bunny.pcd',
        url='https://raw.githubusercontent.com/PointCloudLibrary/pcl/pcl-1.8.0/test/bunny.pcd',
        md5='a4e58778ba12d3f26304127f6be82897',
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/arc2017_4objects.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vakpvU0wtMFNCTkk',
        md5='2c3af4482cd2e0ee95b58848ae48afaf',
    )


if __name__ == '__main__':
    main()
