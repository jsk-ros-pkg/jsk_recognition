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
        path='sample/data/bunny.pcd',
        url='https://raw.githubusercontent.com/PointCloudLibrary/pcl/pcl-1.8.0/test/bunny.pcd',
        md5='a4e58778ba12d3f26304127f6be82897',
    )


if __name__ == '__main__':
    main()
