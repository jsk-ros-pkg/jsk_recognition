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


def cmd_extract_bof(data_path, output):
    print('loading data')
    with gzip.open(data_path, 'rb') as f:
        descs = pickle.load(f)['descriptors']
    X = np.vstack(map(lambda x: np.array(x).reshape((-1, 128)), descs))
    del descs
    # extract feature
    print('fitting bag of features extractor')
    bof = BagOfFeatures()
    bof.fit(X)
    # save bof extractor
    print('saving bof')
    with gzip.open(bof_path, 'wb') as f:
        pickle.dump(bof, f)


def cmd_extract_bof_hist(data_path, bof_path, output):
    print('creating dataset')
    with gzip.open(data_path, 'rb') as f:
        dataset = pickle.load(f)
    descs, y, target_names = (dataset['descriptors'],
                              dataset['target'],
                              dataset['target_names'])
    print('extracting feature')
    with gzip.open(bof_path, 'rb') as f:
        bof = pickle.load(f)
    X = bof.transform(descs)
    normalize(X, copy=False)
    dataset = Bunch(data=X, target=y, target_names=target_names)
    print('saving dataset')
    with gzip.open(bof_hist_path, 'wb') as f:
        pickle.dump(dataset, f)


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command')
    # extract_bof command
    bof_parser = subparsers.add_parser('extract_bof',
        help='fit dataset and extract Bag of Features')
    bof_parser.add_argument('data_path', help='SIFT data path')
    bof_parser.add_argument('-O', '--output', default='bof.pkl.gz',
        help='bof feature extractor instance save path')
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
        cmd_extract_bof(data_path=args.data_path, output=args.output)
    elif args.command == 'extract_bof_hist':
        cmd_extract_bof_hist(data_path=args.data_path,
                             bof_path=args.bof_path,
                             output=args.output)


if __name__ == '__main__':
    main()
