#!/usr/bin/env python

import sys

from xml.dom.minidom import parseString

with open(sys.argv[1]) as f:
    lines = f.readlines()
    text = '<dummy>%s</dummy>' % ('\n'.join(lines))
    root = parseString(text)
    classes = root.getElementsByTagName("class")
    counter = 0
    for klass in classes:
        # Blacklist
        class_name = klass.getAttribute("type")
        if not class_name in ["jsk_pcl_ros::ImageRotateNodelet",
                              "jsk_pcl_ros::MaskImageToDepthConsideredMaskImage",
                              "jsk_pcl_ros::ColorizeRandomForest",
                              "jsk_pcl_ros::Kinfu",
                              "jsk_pcl_ros::ResizePointsPublisher",
                              "jsk_pcl_ros::OrganizedEdgeDetector"]:
            print "%s instance_%d;" % (class_name, counter)
            counter = counter + 1
    
    
