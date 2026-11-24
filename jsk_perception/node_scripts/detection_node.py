#!/usr/bin/env python

import rospy
from jsk_perception.vil_inference_client import DINOClientNode


def main():
    rospy.init_node("dino")
    node = DINOClientNode()
    rospy.spin()

if __name__ == "__main__":
    main()
