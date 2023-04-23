#!/usr/bin/env python

from __future__ import print_function

import argparse
import multiprocessing
import os.path as osp

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

    PKG = 'jsk_perception_3rdparty'

    # remove background (rembg)
    download_data(
        pkg_name=PKG,
        path='trained_data/rembg/u2net/u2net.onnx',
        url='https://drive.google.com/uc?id=1tCU5MM1LhRgGou5OpmpjBQbSrYIUoYab',
        md5='60024c5c889badc19c04ad937298a77b',
        quiet=quiet,
    )


if __name__ == '__main__':
    main()
