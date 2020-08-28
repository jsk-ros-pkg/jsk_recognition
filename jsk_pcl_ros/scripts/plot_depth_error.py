#!/usr/bin/env python

import rospy
import numpy as np
import csv
import sys
import matplotlib.pyplot as plt
from sklearn import linear_model, datasets

if __name__ == "__main__":
    csv_files = sys.argv[1:]
    xs = []
    ys = []
    es = []
    for csv_file in csv_files:
        with open(csv_file) as f:
            reader = csv.reader(f)
            (true_depths, observed_depths) = zip(*[(float(row[0]), float(row[1])) for row in reader])
            errs = [a - b for (a, b) in zip(true_depths, observed_depths)]
            true_mean = np.mean(true_depths)
            observed_mean = np.mean(observed_depths)
            err_mean = np.mean(errs, axis=0)
            err_stddev = np.std(errs, axis=0)
            xs.append(true_mean)
            ys.append(observed_mean)
            #es.append(err_mean)
            es.append(err_stddev)
    #plt.errorbar(xs, ys, es, linestyle='None', marker='^')
    plt.scatter(xs, es)
    model_ransac = linear_model.RANSACRegressor(linear_model.LinearRegression(), min_samples=2,
                                                residual_threshold=0.1)
    X = np.array(xs).reshape((len(xs), 1))
    Y = np.array(es)
    model_ransac.fit(X, Y)
    line_y_ransac = model_ransac.predict(X)
    plt.plot(X, line_y_ransac, "r--",
             label="{0} x + {1}".format(model_ransac.estimator_.coef_[0][0],
                                        model_ransac.estimator_.intercept_[0]))
    print("{0} x + {1}".format(model_ransac.estimator_.coef_[0][0],
                               model_ransac.estimator_.intercept_[0]))
    plt.grid()
    plt.xlabel("Distance [m]")
    plt.ylabel("Standard Deviation [m]")
    # plt.legend(prop={'size': '8'})
    plt.show()
