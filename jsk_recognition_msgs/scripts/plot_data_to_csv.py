#!/usr/bin/env python

import os
import os.path as osp

import rospy
from jsk_recognition_msgs.msg import PlotData
import csv

counter = 0


def callback(msg):
    global counter

    try:
        file_i = osp.expanduser(filename % counter)
    except TypeError:
        file_i = osp.expanduser(filename)
        rospy.loginfo("Output file %s will be overwritten." % file_i)

    rospy.loginfo("writing to %s" % file_i)

    dst_dir = osp.dirname(file_i)
    if not osp.isdir(dst_dir):
        os.makedirs(dst_dir)

    with open(file_i, "w") as f:
        writer = csv.writer(f, delimiter=',')
        writer.writerow(["x", "y"])
        for x, y in zip(msg.xs, msg.ys):
            writer.writerow([x, y])
    rospy.loginfo("done")
    counter = counter + 1


if __name__ == "__main__":
    rospy.init_node("plot_data_to_csv")
    filename = rospy.get_param("~filename", "output_%04d.csv")
    sub = rospy.Subscriber("~input", PlotData, callback, queue_size=1)
    rospy.spin()
