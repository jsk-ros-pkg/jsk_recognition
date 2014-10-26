#!/usr/bin/env python
import cv2
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
import math
import datetime
from jsk_pcl_ros.srv import DepthCalibrationParameter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

xs = []
raw_xs = []
ys = []
us = []
vs = []
c_us = []
c_vs = []
value_cache = dict()            # (u, v) -> [z]
eps_z = 0.03                    # 3cm
lock = threading.Lock()
u_min = None
u_max = None
v_min = None
v_max = None
model = None
set_param = None
MODELS = ["linear", "quadratic", "quadratic-uv", "quadratic-uv-abs"]

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


def getXFromFeatureVector(v):
    if model == "linear":
        return v[0]
    elif model == "quadratic":
        return v[1]
    elif model == "quadratic-uv":
        return v[-3]
    elif model == "quadratic-uv-abs":
        return v[-3]

def genFeatureVector(x, u, v, cu, cv):
    global model
    x2 = x * x
    u2 = u * u
    v2 = v * v
    if model == "linear":
        return [x]
    elif model == "quadratic":
        return [x2, x]
    elif model == "quadratic-uv":
        return [u * x2, v * x2, x2, u * x, v * x, x, u, v]
    elif model == "quadratic-uv-abs":
        u = abs(u - cu)
        v = abs(v - cv)
        return [u * x2, v * x2, x2, u * x, v * x, x, u, v]

def isValidClassifier(classifier):
    # before classifire outputs meaningful value,
    # intercept is a list, so we skip the value
    c0 = classifier.intercept_
    return not hasattr(c0, "__len__");
    
def setParameter(classifier):
    global set_param
    c = classifier.coef_
    c0 = classifier.intercept_
    if not isValidClassifier(classifier):
        print "parameters are list"
        return
    if model == "linear":
        set_param(0, 0, 0,    0, 0, 0,    0, 0, c0, False)
    elif model == "quadratic":
        set_param(0, 0, c[0], 0, 0, c[1], 0, 0, c0, False)
    elif model == "quadratic-uv":
        set_param(c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c0, False)
    elif model == "quadratic-uv-abs":
        set_param(c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c0, True)

def processData(x, y, u, v, cu, cv, fit = True):
    global xs, ys, classifier, u_min, u_max, v_min, v_max, raw_xs
    with lock:
        if value_cache.has_key((u, v)):
            zs = value_cache[(u, v)]
            for z in zs:
                if abs(z - y) < eps_z:
                    print "duplicated value"
                    return
                else:
                    value_cache[(u, v)].append(y)
        else:
            value_cache[(u, v)] = [y]
        raw_xs.append(x)
        us.append(u)
        vs.append(v)
        c_us.append(cu)
        c_vs.append(cv)
        if u > u_min and u < u_max and v < v_max and v > v_min:
            print (x, y)
            xs.append(genFeatureVector(x, u, v, cu, cv))
            ys.append(y)
            if fit:
                classifier.fit(xs, ys)
                try:
                    setParameter(classifier)
                except rospy.service.ServiceException, e:
                    rospy.logfatal("failed to call service: %s" % (e.message))
            try:
                print modelEquationString(classifier)
            except Exception, e:
                rospy.logwarn("failed to print model: %s" % e.message)
            
            else:
                print "(%d, %d) is out of range" % (u, v)
        
def callback(msg):
    global xs, ys, classifier, u_min, u_max, v_min, v_max, raw_xs
    
    x = msg.observed_depth
    y = msg.true_depth
    u = msg.u
    v = msg.v
    if math.isnan(x) or math.isnan(y):
        return
    processData(x, y, u, v, msg.center_u, msg.center_v)

def uvCoefString(c, absolute=False):
    if absolute:
        return "%f|u| + %f|v| + %f" % (c[0], c[1], c[2])
    else:
        return "%fu + %fv + %f" % (c[0], c[1], c[2])
        
def modelEquationString(classifier):
    global model, xs, ys
    c = classifier.coef_
    i = classifier.intercept_
    if model == "linear":
        return "%fz + %f\n(score: %f)" % (
            c[0], i,
            classifier.score(xs, ys))
    elif model == "quadratic":
        return "%fz^2 + %fz + %f\n(score: %f)" % (
            c[0],
            c[1],
            i,
            classifier.score(xs, ys))
    elif model == "quadratic-uv":
        return "(%s)z^2 +\n(%s)z +\n%s\n(score: %f)" % (
            uvCoefString(c[0:3]),
            uvCoefString(c[3:6]),
            uvCoefString([c[6], c[7], i]),
            classifier.score(xs, ys))
    elif model == "quadratic-uv-abs":
        return "(%s)z^2 +\n(%s)z +\n%s\n(score: %f)" % (
            uvCoefString(c[0:3], True),
            uvCoefString(c[3:6], True),
            uvCoefString([c[6], c[7], i], True),
            classifier.score(xs, ys))

def applyModel(x, u, v, cu, cv, clssifier):
    global model
    c = classifier.coef_
    i = classifier.intercept_
    if model == "linear":
        return c[0] * x + i
    elif model == "quadratic":
        return c[0] * x * x + c[1] * x + i
    elif model == "quadratic-uv":
        return (c[0] * u + c[1] * v + c[2]) * x * x + (c[3] * u + c[4] * v + c[5]) * x + c[6] * u + c[7] * v + i
    elif model == "quadratic-uv-abs":
        u = abs(u - cu)
        v = abs(v - cv)
        return (c[0] * u + c[1] * v + c[2]) * x * x + (c[3] * u + c[4] * v + c[5]) * x + c[6] * u + c[7] * v + i

def updatePlot(num):
    global xs, ax, width, height
    if rospy.is_shutdown():
        plt.close()
        return
    with lock:
        if len(xs) == 0:
            return
        try:
            plt.cla()
            plt.xlabel('Z from depth image')
            plt.ylabel('Z from checker board')
            plt.grid(True)
            plt.title(model)
            plt.scatter([getXFromFeatureVector(x) for x in xs[:-1]],
                        ys[:-1], s=10, c='b', zorder=10, alpha=0.1)
            plt.scatter([getXFromFeatureVector(xs[-1])],
                        [ys[-1]], s=10, c='r', zorder=10, alpha=1.0)
            xmin = np.amin([getXFromFeatureVector(x) for x in xs])
            xmax = np.amax([getXFromFeatureVector(x) for x in xs])
            X_test = [[xmin * xmin, xmin], [xmax * xmax, xmax]]
            X_range = np.linspace(xmin, xmax, 100)
            ax.plot(X_range, X_range, linewidth=2, color='green', alpha=0.5)
            ax.plot(X_range, 
                    applyModel(X_range, 320, 240, 320, 240, classifier),
                    linewidth=2, color='red', alpha=0.5)
            plt.text(xmin, xmax - 0.1,
                     modelEquationString(classifier),
                     fontsize=12)
            # publish frequency image
            bridge = CvBridge()
            img = generateFrequencyMap()
            pub_image.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
        except Exception, e:
            rospy.logerr(e.message)

def generateFrequencyMap():
    # bgr
    img = np.tile(np.uint8([0,0,0]), (height, width, 1))
    for (u, v) in value_cache.keys():
        # max = 100
        min_color = np.uint8([255, 0, 0])
        max_color = np.uint8([0, 0, 255])
        r = min(len(value_cache[(u, v)]) / 10.0, 1)
        img[v, u] = min_color * (1 - r) + max_color * r
    return img
        
def main():
    global ax, xs, ys, classifier, u_min, u_max, v_min, v_max, model, set_param
    global width, height, pub_image
    width = 640
    height = 480
    pub_image = rospy.Publisher("~frequency_image", Image)
    set_param = rospy.ServiceProxy("depth_calibration/set_calibration_parameter", 
                                   DepthCalibrationParameter)
    # parse argument
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv')
    parser.add_argument('--model', default="linear")
    parser.add_argument('--models', action="store_true", default=False)
    args = parser.parse_args(rospy.myargv()[1:])
    if args.models:
        for m in MODELS:
            print m
        return
    model = args.model
    if model not in MODELS:
        raise Exception("Unknown Model: %s" % (model))
    
    if not args.csv:
        sub = rospy.Subscriber("depth_image_error/output", 
                               DepthErrorResult, callback)
        #plt.ion()
    fig = plt.figure()
    ax = plt.axes([.12, .12, .8, .8])
    anim = animation.FuncAnimation(fig, updatePlot)
    classifier = linear_model.LinearRegression()
    u_min = rospy.get_param("~u_min", 0)
    u_max = rospy.get_param("~u_max", 4096)
    v_min = rospy.get_param("~v_min", 0)
    v_max = rospy.get_param("~v_max", 4096)
    
    if args.csv:
        for row in csv.reader(open(args.csv, "rb")):
            x = float(row[0])
            y = float(row[1])
            u = float(row[2])
            v = float(row[3])
            cu = float(row[4])
            cv = float(row[5])
            processData(x, y, u, v, cu, cv, fit = False)
        classifier.fit(xs, ys)
    try:
        plt.show()
    finally:
        if not args.csv:
            csv_filename = "calibration-%s.csv" % datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
            print "Save calibration parameters to %s" % (csv_filename)
            with open(csv_filename, "w") as f:
                for x, y, u, v, cu, cv in zip(raw_xs, ys, us, vs, c_us, c_vs):
                    f.write("%f,%f,%d,%d,%f,%f\n" % (x, y, u, v, cu, cv))
        if query_yes_no("Dump result into yaml file?"):
            print "writing to calibration_parameter.launch"
            c = classifier.coef_
            if model == "quadratic-uv-abs":
                use_abs = "True"
            else:
                use_abs = "False"
            yaml_filename = "calibration_parameter_%s.yaml" % datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
            with open(yaml_filename, "w") as f:
                f.write("""
coefficients2: [%f, %f, %f]
coefficients1: [%f, %f, %f]
coefficients0: [%f, %f, %f]
use_abs: %s
                """ % (c[0], c[1], c[2], 
                       c[3], c[4], c[5], 
                       c[6], c[7], classifier.intercept_,
                       use_abs))

if __name__ == "__main__":
    rospy.init_node("depth_error_logistic_regression")
    main()
