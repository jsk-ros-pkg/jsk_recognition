#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
import os
import sys
import gzip
import cPickle as pickle
import argparse

import numpy as np
from sklearn.preprocessing import normalize
from sklearn.linear_model import LogisticRegression
from sklearn.cross_validation import train_test_split
from sklearn.metrics import classification_report, accuracy_score


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset',
        help='dataset must have data, target, target_names attributes')
    parser.add_argument('-c', '--classifier', default='logistic_regression',
                        help='now supports logistic_regression only')
    parser.add_argument('-O', '--output', default='clf.pkl.gz',
                        help='saving clf filename')
    args = parser.parse_args(sys.argv[1:])

    print('loading dataset')
    with gzip.open(args.dataset, 'rb') as f:
        dataset = pickle.load(f)

    X = dataset.data
    y = dataset.target
    target_names = dataset.target_names

    # create train and test data
    X_train, X_test, y_train, y_test = train_test_split(X, y,
                                        random_state=np.random.randint(1234))

    # train and test
    if args.classifier == 'logistic_regression':
        clf = LogisticRegression()
    else:
        raise ValueError('unsupported classifier')
    print(('fitting {0}'.format(args.classifier)))
    clf.fit(X_train, y_train)
    clf.target_names_ = target_names
    with gzip.open(args.output, 'wb') as f:
        pickle.dump(clf, f)
    y_pred = clf.predict(X_test)
    print(('score of classifier: {}'.format(accuracy_score(y_test, y_pred))))
    print((classification_report(y_test, y_pred, target_names=target_names)))


if __name__ == '__main__':
    main()
