#!/usr/bin/env python3

from jsk_data import download_data


def main():
    PKG = 'image_restoration'

    download_data(
        pkg_name=PKG,
        path='sample/data/sample_bag_for_image_restoration.bag',
        url='https://drive.google.com/file/d/10ainoVwn_jwGmmRsMvneglPwgGSrmlvD/view?usp=sharing',
        md5='aa65107c303b5c7325074eb4d53c8d95',
        extract=False,
    )

if __name__ == '__main__':
    main()
