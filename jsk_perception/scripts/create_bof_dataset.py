#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Create BoF and BoF Histogram dataset
"""

import gzip
import cPickle as pickle
import argparse

import numpy as np
from sklearn.datasets.base import Bunch
from sklearn.preprocessing import normalize
from jsk_recognition_utils import BagOfFeatures


def cmd_extract_bof(data_path, output, data_size=1):
    print('loading data')
    with gzip.open(data_path, 'rb') as f:
        descs = pickle.load(f)['descriptors']
    # limit to specified data size to avoid memory error
    n_data_all = len(descs)
    n_data = int(data_size * n_data_all)
    p = np.random.randint(0, len(descs), n_data)
    descs = np.array(descs)[p]
    X = np.vstack(map(lambda x: np.array(x).reshape((-1, 128)), descs))
    del descs
    # extract feature
    print('fitting bag of features extractor')
    bof = BagOfFeatures()
    try:
        bof.fit(X)
    except MemoryError as e:
        print(('data_size: {} ({} * {})'.format(n_data, data_size, n_data_all)))
        print(e)
    # save bof extractor
    print('saving bof')
    with gzip.open(output, 'wb') as f:
        pickle.dump(bof, f)
    print('done')


def cmd_extract_bof_hist(data_path, bof_path, output):
    print('creating dataset')
    with gzip.open(data_path, 'rb') as f:
        dataset = pickle.load(f)
    descs, y, target_names = (dataset['descriptors'],
                              dataset['target'],
                              dataset['target_names'])
    del dataset
    print('extracting feature')
    with gzip.open(bof_path, 'rb') as f:
        bof = pickle.load(f)
    X = bof.transform(descs)
    del descs
    normalize(X, copy=False)
    dataset = Bunch(data=X, target=y, target_names=target_names)
    print('saving dataset')
    with gzip.open(output, 'wb') as f:
        pickle.dump(dataset, f)
    print('done')


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command')
    # extract_bof command
    bof_parser = subparsers.add_parser('extract_bof',
        help='fit dataset and extract Bag of Features')
    bof_parser.add_argument('data_path', help='SIFT data path')
    bof_parser.add_argument('-O', '--output', default='bof.pkl.gz',
        help='bof feature extractor instance save path')
    bof_parser.add_argument('-s', '--data-size', default=1, type=float,
        help='data_size in 0 to 1')
    # extract_bof_hist command
    dataset_parser = subparsers.add_parser('extract_bof_hist',
        help='create BoF histogram dataset')
    dataset_parser.add_argument('data_path', help='SIFT data path')
    dataset_parser.add_argument('bof_path',
        help="BoF data path extracted by 'extract_bof' command")
    dataset_parser.add_argument('-O', '--output', default='bof_hist.pkl.gz',
        help='save path of bof histogram (default: bof_hist.pkl.gz)')
    args = parser.parse_args()

    if args.command == 'extract_bof':
        cmd_extract_bof(data_path=args.data_path,
                        output=args.output,
                        data_size=args.data_size)
    elif args.command == 'extract_bof_hist':
        cmd_extract_bof_hist(data_path=args.data_path,
                             bof_path=args.bof_path,
                             output=args.output)


if __name__ == '__main__':
    main()
