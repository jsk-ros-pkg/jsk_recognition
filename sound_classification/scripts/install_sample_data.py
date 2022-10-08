#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'sound_classification'

    download_data(
        pkg_name=PKG,
        path='sample/data/room610-sound-2022-01-05-11-02-05.bag',
        url='https://drive.google.com/uc?id=1DLpTWfx14Vc_A23rpr2SKiWqGvvzTvpv',
        md5='92d04717e85fffc8ca518811f90a10b6',
        extract=False,
        compressed_bags=[
            'sample/data/room610-sound-2022-01-05-11-02-05.bag',
        ],
    )


if __name__ == '__main__':
    main()
