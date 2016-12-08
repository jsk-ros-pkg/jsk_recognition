#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'jsk_pcl_ros_utils'

    download_data(
        pkg_name=PKG,
        path='sample/data/2015-11-04-19-37-29_baxter-kiva-object-in-hand-cloud.tgz',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vSE5XMmJQczl3NWc',
        md5='4da6242d74950a171e6d4455be87932e',
        extract=True,
        compressed_bags=[
            'sample/data/2015-11-04-19-37-29_baxter-kiva-object-in-hand-cloud/vision.compressed.bag',
        ],
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/bunny.pcd',
        url='https://raw.githubusercontent.com/PointCloudLibrary/pcl/pcl-1.8.0/test/bunny.pcd',
        md5='a4e58778ba12d3f26304127f6be82897',
    )


if __name__ == '__main__':
    main()
