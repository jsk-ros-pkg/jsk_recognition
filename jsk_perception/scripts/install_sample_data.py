#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'jsk_perception'

    download_data(
        pkg_name=PKG,
        path='sample/data/2016-11-11-12-53-06_in_lab.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vdE9IRERDLXRoUWs',
        md5='5066157f7373a19829f4edba65068a89',
        extract=False,
        compressed_bags=[
            'sample/data/2016-11-11-12-53-06_in_lab.bag',
        ],
    )

    download_data(
        pkg_name=PKG,
        path='sample/data/2016-10-15-23-21-42_moving_bottle.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vTDZOZGhrTTNvc00',
        md5='ad4e7d298c0d9985295d93e47c7b03e6',
        extract=False,
        compressed_bags=[
            'sample/data/2016-10-15-23-21-42_moving_bottle.bag',
        ],
    )


if __name__ == '__main__':
    main()
