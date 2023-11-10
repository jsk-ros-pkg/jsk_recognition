#!/usr/bin/env python

import rospy
from jsk_perception.vil_inference_client import ClipClientNode


def main():
    rospy.init_node("classification")
    node = ClipClientNode()
    rospy.spin()

if __name__ == "__main__":
    main()
