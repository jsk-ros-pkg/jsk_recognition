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

xs = []
ys = []
lock = threading.Lock()


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

def callback(msg):
    global xs, ys, classifier
    with lock:
        x = msg.observed_depth
        y = msg.true_depth
        print (x, y)
        xs.append([x])
        ys.append(y)
        classifier.fit(xs, ys)
    
def main():
    global ax, xs, ys, classifier
    plt.ion()
    fig = plt.figure()
    ax = plt.axes([.12, .12, .8, .8])
    sub = rospy.Subscriber("depth_image_error/output", DepthErrorResult, callback)
    r = rospy.Rate(10)
    classifier = linear_model.LinearRegression()
    try:
        while not rospy.is_shutdown():
            if len(xs) == 0:
                continue
            with lock:
                xmin = np.amin(xs)
                xmax = np.amax(xs)
                X_test = np.c_[xmin, xmax].T
                plt.cla()
                ax.set_xlabel('Z from depth image')
                ax.set_ylabel('Z from checker board')
                ax.scatter(xs, ys, s=10, c='b', zorder=10, alpha=0.5)
                ax.plot(X_test, classifier.predict(X_test), linewidth=2, color='red', alpha=0.5)
                ax.plot(X_test, X_test, linewidth=2, color='green', alpha=0.5)
                plt.text(xmin, xmax,
                         "%fz + %f (score: %f)" 
                % (classifier.coef_[0], classifier.intercept_, classifier.score(xs, ys)),
                fontsize=12)
                plt.draw()
    finally:
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
