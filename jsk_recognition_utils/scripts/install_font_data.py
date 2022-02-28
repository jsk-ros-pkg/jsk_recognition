#!/usr/bin/env python

import argparse
import multiprocessing

import jsk_data


def download_data(*args, **kwargs):
    p = multiprocessing.Process(
            target=jsk_data.download_data,
            args=args,
            kwargs=kwargs)
    p.start()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    quiet = args.quiet

    PKG = 'jsk_recognition_utils'

    download_data(
        pkg_name=PKG,
        path='font_data/NotoSansJP-Regular.otf',
        url='https://github.com/googlefonts/noto-cjk/raw/main/Sans/OTF/Japanese/NotoSansCJKjp-Regular.otf',  # NOQA
        md5='6d57a40c6695bd46457315e2a9dc757a',
        quiet=quiet,
    )


if __name__ == '__main__':
    main()
