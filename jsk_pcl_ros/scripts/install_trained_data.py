#!/usr/bin/env python

import argparse
import multiprocessing

import jsk_data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    quiet = args.quiet

    def download_data(**kwargs):
        kwargs['pkg_name'] = 'jsk_pcl_ros'
        kwargs['quiet'] = quiet
        p = multiprocessing.Process(
            target=jsk_data.download_data,
            kwargs=kwargs)
        p.start()

    download_data(
        path='trained_data/linemod_template.tgz',
        url='https://drive.google.com/uc?id=1nQzjrpvLojzPrQDWElxDCTVjLzeDi70p',
        md5='2c9bd31c6c6ddd5f36698fb36040c71c',
        extract=True,
    )


if __name__ == '__main__':
    main()
