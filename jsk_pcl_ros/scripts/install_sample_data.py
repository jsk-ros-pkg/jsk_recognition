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

    download_data(
        pkg_name=PKG,
        path='test_data/2016-10-29-17-05-44_tabletop-coffee-cup.tgz',
        url='https://drive.google.com/uc?id=0B5DV6gwLHtyJczRRcUhURndQeDg',
        md5='d58e894c853fc4485d08547b9fc2b64',
        extract=True,
        compressed_bags=[
            'test_data/2016-10-29-17-05-44_tabletop-coffee-cup/tabletop-coffee-cup.bag',
        ],
    )

    download_data(
        pkg_name=PKG,
        path='test_data/2016-10-26-02-09-51_coffee_cup.pcd',
        url='https://drive.google.com/uc?id=0B5DV6gwLHtyJVVQ2TFhVSTJqZ3M',
        md5='27d1a39e6ea596c4e24f1347f53f2e7b',
        extract=False,
    )


if __name__ == '__main__':
    main()
