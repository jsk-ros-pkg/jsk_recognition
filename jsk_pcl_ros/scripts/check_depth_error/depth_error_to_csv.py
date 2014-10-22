#!/usr/bin/env python
import os
import rospy
from jsk_pcl_ros.msg import DepthErrorResult

data = []

def errorCallback(msg):
    global data
    print "New Message"
    data.append((msg.observed_depth, msg.true_depth))

def main():
    global data
    s = rospy.Subscriber("~input", DepthErrorResult, errorCallback)
    try:
        rospy.spin()
    finally:
        sorted_data = sorted(data, key=lambda p: p[0])
        output_file = os.path.abspath("output.csv")
        print "writing depth error to %s" % (output_file)
        with open(output_file, "w") as f:
            f.write("depth_z,rgb_z\n")
            for p in sorted_data:
                f.write("%f,%f\n" % (p[0], p[1]))
    
if __name__ == "__main__":
    rospy.init_node("depth_error_to_csv")
    main()
