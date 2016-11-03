#!/usr/bin/env python

from jsk_data import download_data

def main():
    PKG = 'jsk_pcl_ros'
    
    download_data(
        pkg_name=PKG,
        path='test_data/2015-11-04-19-37-29_baxter-kiva-object-in-hand-cloud.tgz',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vSE5XMmJQczl3NWc',
        md5='4da6242d74950a171e6d4455be87932e',
        extract=True,
        compressed_bags=[
            'test_data/2015-11-04-19-37-29_baxter-kiva-object-in-hand-cloud/vision.compressed.bag',
        ],
    )

    download_data(
        pkg_name=PKG,
        path='test_data/drill.pcd',
        url='http://www.jsk.t.u-tokyo.ac.jp/~ueda/dataset/2015/02/drill.pcd',
        md5='b6ea8f7bd97fd1e88fb6af2f6cd42ac5',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='test_data/2016-07-06-12-16-43-person-in-point-cloud.tgz',
        url='https://drive.google.com/uc?id=0B_NiLAzvehC9cHhnOFd0YUN2N1U',
        md5='c59067adc429fb9f1cf180d350a2da43',
        extract=True,
        compressed_bags=[
            'test_data/2016-07-06-12-16-43-person-in-point-cloud/vision.compressed.bag',
        ],
    )

if __name__ == '__main__':
    main()
