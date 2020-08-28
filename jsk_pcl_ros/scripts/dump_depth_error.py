#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import DepthErrorResult, PlotData
import numpy as np
import csv

grid = 0.1                     # 10 cm
bins = {}

data = PlotData()

def callback(msg):
    bin_index = int(msg.true_depth / grid)
    if not bins.has_key(bin_index):
        bins[bin_index] = []
    err = msg.true_depth - msg.observed_depth
    bins[bin_index].append((msg, err))
    for bin_index, msg_and_errs in bins.items():
        if len(msg_and_errs) < 5:
            continue
        print("Bin: {0}m ~ {1}m".format(grid * bin_index, grid * (bin_index + 1)))
        print("   Sample:", len(msg_and_errs))
        errs = [err for (msg, err) in msg_and_errs]
        mean = np.mean(errs, axis=0)
        stddev = np.std(errs, axis=0)
        print("   Mean:", mean)
        print("   Stddev:", stddev)
    data.xs.append(msg.true_depth)
    data.ys.append(msg.observed_depth)
    pub.publish(data)
    writer.writerow([msg.true_depth, msg.observed_depth])
        

if __name__ == "__main__":
    rospy.init_node("plot_depth_error")
    csv_path = rospy.get_param("~csv_path", "output.csv")
    pub = rospy.Publisher("~scatter", PlotData, queue_size=1)
    f = open(csv_path, "w")
    writer = csv.writer(f)
    sub = rospy.Subscriber("/depth_image_error/output", DepthErrorResult, callback)
    rospy.spin()
    
