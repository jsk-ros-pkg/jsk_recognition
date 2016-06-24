#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'jsk_pcl_ros'

    download_data(
        pkg_name=PKG,
        path='test_data/2016-06-24-17-43-57_tabletop.bag.tgz',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vUllPT3VPOHN6LVU',
        md5='dbc3daff4fe9812cab7328e7702e6438',
        extract=True,
    )


if __name__ == '__main__':
    main()
