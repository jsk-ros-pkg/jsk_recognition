#!/usr/bin/env python

import random

import matplotlib
matplotlib.use('Agg')  # NOQA
from matplotlib import patches
import matplotlib.pyplot as plt
import numpy as np

import cv_bridge
import rospy
from sensor_msgs.msg import Image

try:
    from ml_classifiers.srv import *
    from ml_classifiers.msg import *
except:
    import roslib;roslib.load_manifest("ml_classifiers")
    from ml_classifiers.srv import *
    from ml_classifiers.msg import *


HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'


if __name__ == "__main__":
    rospy.init_node("random_forest_client")
    br = cv_bridge.CvBridge()
    pub_img = rospy.Publisher('~output/debug_image', Image, queue_size=1)
    rospy.wait_for_service('predict')
    rospy.loginfo("Start Request Service!!")

    predict_data = rospy.ServiceProxy('predict', ClassifyData)

    old_targets_ok = []
    old_targets_fail = []

    while not rospy.is_shutdown():
        req = ClassifyDataRequest()
        req_point = ClassDataPoint()
        target = [random.random(), random.random()]
        answer = 1
        #Check if it is in the circle radius = 1?
        if target[0] * target[0] + target[1] * target[1] > 1:
            answer = 0
        req_point.point = target
        req.data.append(req_point)
        print(OKGREEN,"Send Request         ====================>         Answer",ENDC)
        print(OKGREEN,"    ",req_point.point,"       : ",str(answer),ENDC)
        response = predict_data(req)
        print(WARNING,"Get the result : ",ENDC)
        print(WARNING,response.classifications,ENDC)
        succeed = int(float(response.classifications[0])) == answer
        if succeed:
            print(OKBLUE,"Succeed!!!",ENDC)
        else:
            print(FAIL,"FAIL...",FAIL)
        print("--- --- --- ---")

        # Config for plotting
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.set_aspect('equal')
        ax.grid()
        ax.set_xlim(-0.1, 1.1)
        ax.set_ylim(-0.1, 1.1)
        ax.set_xticks([0.0, 0.5, 1.0])
        ax.set_yticks([0.0, 0.5, 1.0])
        ax.set_title('Random Forest Classification Result')

        # Draw boundary
        circle = patches.Circle(xy=(0, 0), radius=1.0, fill=False, ec='g')
        ax.add_patch(circle)

        # Plot old classification result
        old_targets_ok_nparr = np.array(old_targets_ok)
        old_targets_fail_nparr = np.array(old_targets_fail)
        if old_targets_ok_nparr.size > 0:
            ax.plot(old_targets_ok_nparr[:, 0], old_targets_ok_nparr[:, 1],
                    'bo', label='Successfully classified')
        if old_targets_fail_nparr.size > 0:
            ax.plot(old_targets_fail_nparr[:, 0], old_targets_fail_nparr[:, 1],
                    'rx', label='Failed to classify')

        # Plot current classification result
        if succeed:
            ax.plot(target[0], target[1], 'bo', markersize=12)
            old_targets_ok.append(target)
        else:
            ax.plot(target[0], target[1], 'rx', markersize=12)
            old_targets_fail.append(target)

        # Update and publish the image.
        # Somehow we have to call tight_layout() some times
        # to keep the legend from protruding from the image.
        ax.legend(bbox_to_anchor=(1.05, 0.5), loc='center left')
        for i in range(3):
            fig.tight_layout()
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8)
        img.shape = (h, w, 3)
        fig.clf()
        plt.close()
        img_msg = br.cv2_to_imgmsg(img, 'rgb8')
        pub_img.publish(img_msg)

        rospy.sleep(1)
