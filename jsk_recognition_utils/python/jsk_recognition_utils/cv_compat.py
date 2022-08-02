import importlib
import os
import sys


# OpenCV import for python3
if os.environ['ROS_PYTHON_VERSION'] == '3':
    cv2 = importlib.import_module("cv2")
else:
    sys.path.remove('/opt/ros/{}/lib/python2.7/dist-packages'
                    .format(os.getenv('ROS_DISTRO')))
    cv2 = importlib.import_module("cv2")
    sys.path.append('/opt/ros/{}/lib/python2.7/dist-packages'
                    .format(os.getenv('ROS_DISTRO')))
