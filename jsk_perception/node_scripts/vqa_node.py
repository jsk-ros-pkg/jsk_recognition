#!/usr/bin/env python

import rospy
from jsk_perception.vil_inference_client import OFAClientNode


def main():
    rospy.init_node("vqa")
    node = OFAClientNode()
    rospy.spin()

if __name__ == "__main__":
    main()
