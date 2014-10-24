#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from jsk_pcl_ros.msg import DepthErrorResult
from sklearn import linear_model
import time
import threading
import sys
import rospy
import argparse
import csv

xs = []
ys = []
lock = threading.Lock()
u_min = None
u_max = None
v_min = None
v_max = None
model = None

def query_yes_no(question, default=None):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).

    The "answer" return value is one of "yes" or "no".
    """
    valid = {"yes": True, "y": True, "ye": True,
             "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = raw_input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "
                             "(or 'y' or 'n').\n")


def genFeatureVector(x):
    global model
    if model == "linear":
        return [x]
    elif model == "quadratic":
        return [x * x, x]
    
def callback(msg):
    global xs, ys, classifier, u_min, u_max, v_min, v_max
    with lock:
        x = msg.observed_depth
        y = msg.true_depth
        u = msg.u
        v = msg.v
        if u > u_min and u < u_max and v < v_max and v > v_min:
            print (x, y)
            xs.append(genFeatureVector(x))
            ys.append(y)
            classifier.fit(xs, ys)
        else:
            print "(%d, %d) is out of range" % (u, v)

def modelEquationString(classifier):
    global model, xs, ys
    if model == "linear":
        return "%fz + %f (score: %f)" % (classifier.coef_[0], classifier.intercept_,
                                         classifier.score(xs, ys))
    elif model == "quadratic":
        return "%fz^2 %fz + %f (score: %f)" % (classifier.coef_[0],
                                   classifier.coef_[1],
                                   classifier.intercept_,
                                   classifier.score(xs, ys))
            
def applyModel(x, clssifier):
    global model
    if model == "linear":
        return classifier.coef_[0] * x + classifier.intercept_
    elif model == "quadratic":
        return classifier.coef_[0] * x * x + classifier.coef_[1] * x + classifier.intercept_
    
def main():
    global ax, xs, ys, classifier, u_min, u_max, v_min, v_max, model
    # parse argument
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv')
    parser.add_argument('--model', default="linear")
    args = parser.parse_args(rospy.myargv()[1:])
    model = args.model
    if model not in ["linear", "quadratic"]:
        raise Exception("Unknown Model: %s" % (model))
    
    if not args.csv:
        sub = rospy.Subscriber("depth_image_error/output", 
                               DepthErrorResult, callback)
    plt.ion()
    fig = plt.figure()
    ax = plt.axes([.12, .12, .8, .8])
    r = rospy.Rate(10)
    classifier = linear_model.LinearRegression()
    u_min = rospy.get_param("~u_min", 0)
    u_max = rospy.get_param("~u_max", 4096)
    v_min = rospy.get_param("~v_min", 0)
    v_max = rospy.get_param("~v_max", 4096)
    if args.csv:
        for row in csv.reader(open(args.csv, "rb")):
            x = float(row[0])
            y = float(row[1])
            xs.append(genFeatureVector(x))
            ys.append(y)
        classifier.fit(xs, ys)
    try:
        while not rospy.is_shutdown():
            r.sleep()
            if len(xs) == 0:
                continue
            with lock:
                xmin = np.amin([x[-1] for x in xs])
                xmax = np.amax([x[-1] for x in xs])
                #X_test = np.c_[xmin, xmax].T
                X_test = [[xmin * xmin, xmin], [xmax * xmax, xmax]]
                X_range = np.linspace(xmin, xmax, 100)
                plt.cla()
                ax.set_xlabel('Z from depth image')
                ax.set_ylabel('Z from checker board')
                ax.scatter([[x[-1]] for x in xs], 
                           ys, s=10, c='b', zorder=10, alpha=0.5)
                ax.plot(X_range, 
                        applyModel(X_range, classifier),
                        linewidth=2, color='red', alpha=0.5)
                ax.plot(X_range, X_range, linewidth=2, color='green', alpha=0.5)
                plt.text(xmin, xmax,
                         modelEquationString(classifier),
                         fontsize=12)
                plt.draw()
    finally:
        if not args.csv:
            print "Save calibration parameters to calibrate.csv"
            with open("calibrate.csv", "w") as f:
                for x, y in zip(xs, ys):
                    f.write("%f,%f\n" % (x[0], y))
        if query_yes_no("Dump result into launch file?"):
            print "writing to calibration_parameter.launch"
            with open("calibration_parameter.launch", "w") as f:
                f.write("""<launch>
    <node pkg="dynamic_reconfigure" type="dynparam" 
          name="set_z_offset" 
          args="set driver z_offset_mm %d" />
    <node pkg="dynamic_reconfigure" type="dynparam" 
          name="set_z_scale" 
          args="set driver z_scaling %f" />
</launch>
""" % (int(1000.0*classifier.intercept_), classifier.coef_[0]))

if __name__ == "__main__":
    rospy.init_node("depth_error_logistic_regression")
    main()
