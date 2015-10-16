#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Extract Bag of Features and create BoF Histograms

    usage: extract_bof.py [-h] {fit,extract,dataset} ...

    positional arguments:
    {fit,extract,dataset}
        fit                 fit feature extractor using dataset.
        dataset             create bof histogram dataset.
        extract             extract feature in realtime.

"""

import os
import sys
import gzip
import cPickle as pickle
import argparse

import progressbar
import numpy as np
from sklearn.datasets.base import Bunch
from sklearn.preprocessing import normalize
from sklearn.neighbors import NearestNeighbors
from sklearn.cluster import MiniBatchKMeans

import rospy
from posedetection_msgs.msg import Feature0D
from jsk_recognition_msgs.msg import Histogram


class BagOfFeatures(object):
    def __init__(self, hist_size=500):
        self.nn = None
        self.hist_size = hist_size

    def fit(self, X):
        """Fit features and extract bag of features"""
        k = self.hist_size
        km = MiniBatchKMeans(n_clusters=k, init_size=3*k, max_iter=300)
        km.fit(X)
        nn = NearestNeighbors(n_neighbors=1)
        nn.fit(km.cluster_centers_)
        self.nn = nn

    def transform(self, X):
        return np.vstack([self.make_hist(xi.reshape((-1, 128))) for xi in X])

    def make_hist(self, descriptors):
        """Make histogram for one image"""
        nn = self.nn
        if nn is None:
            raise ValueError('must fit features before making histogram')
        indices = nn.kneighbors(descriptors, return_distance=False)
        histogram = np.zeros(self.hist_size)
        for idx in np.unique(indices):
            mask = indices == idx
            histogram[idx] = mask.sum()  # count the idx
            indices = indices[mask == False]
        return histogram


def create_dataset(data_path, bof_path, bof_hist_path):
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


def fit_bof_extractor(data_path, bof_path):
    print('loading data')
    with gzip.open(data_path, 'rb') as f:
        dataset = pickle.load(f)
    descs = dataset['descriptors']
    X = np.vstack(map(lambda x: np.array(x).reshape((-1, 128)), descs))
    # extract feature
    print('fitting bag of features extractor')
    bof = BagOfFeatures()
    bof.fit(X)
    # save bof extractor
    print('saving bof')
    with gzip.open(bof_path, 'wb') as f:
        pickle.dump(bof, f)


class ExtractInRealtime(object):
    def __init__(self, bof_path):
        with gzip.open(bof_path, 'rb') as f:
            self.bof = pickle.load(f)
        self.pub = rospy.Publisher('~output/bof_hist', Histogram, queue_size=1)
        rospy.Subscriber('Feature0D', Feature0D, self._cb_feature0d)
        rospy.loginfo('Initialized bof extractor')

    def _cb_feature0d(self, msg):
        desc = np.array(msg.descriptors)
        X = self.bof.transform([desc])
        normalize(X, copy=False)
        self.pub.publish(Histogram(header=msg.header, histogram=X[0]))


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command')
    # fit command
    fit_parser = subparsers.add_parser('fit',
                        help='fit feature extractor using dataset')
    fit_parser.add_argument('data_path', help='data path')
    fit_parser.add_argument('-O', '--output', default='bof.pkl.gz',
                            help='bof feature extractor instance save path')
    # dataset command
    dataset_parser = subparsers.add_parser('dataset',
                        help='create bof histogram dataset')
    dataset_parser.add_argument('data_path', help='data path')
    dataset_parser.add_argument('bof_path', help='bof data path')
    dataset_parser.add_argument('-O', '--output', default='bof_hist.pkl.gz',
        help='save path of bof histogram (default: bof_hist.pkl.gz)')
    # extract command
    extract_parser = subparsers.add_parser('extract',
                        help='extract feature in realtime')
    extract_parser.add_argument('bof_path', help='bof data path')
    args = parser.parse_args(rospy.myargv(sys.argv[1:]))

    if args.command == 'fit':
        fit_bof_extractor(data_path=args.data_path, bof_path=args.output)
    elif args.command == 'dataset':
        create_dataset(data_path=args.data_path,
                       bof_path=args.bof_path,
                       bof_hist_path=args.output)
    elif args.command == 'extract':
        rospy.init_node('extract_bof')
        ex_real = ExtractInRealtime(bof_path=args.bof_path)
        rospy.spin()


if __name__ == '__main__':
    main()
