#!/usr/bin/env python

try:
    from ml_classifiers.srv import *
    from ml_classifiers.msg import *
except:
    import roslib;roslib.load_manifest("ml_classifiers")
    from ml_classifiers.srv import *
    from ml_classifiers.msg import *

import rospy
import random

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'

if __name__ == "__main__":
    rospy.init_node("random_forest_client")

    rospy.wait_for_service('predict')

    rospy.loginfo("Start Request Service!!")

    predict_data = rospy.ServiceProxy('predict', ClassifyData)

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
        print OKGREEN,"Send Request         ====================>         Answer",ENDC
        print OKGREEN,"    ",req_point.point,"       : ",str(answer),ENDC
        response = predict_data(req)
        print WARNING,"Get the result : ",ENDC
        print WARNING,response.classifications,ENDC
        if response.classifications[0].find(str(answer)):
            print OKBLUE,"Succeed!!!",ENDC
        else:
            print FAIL,"FAIL...",FAIL
        print "--- --- --- ---"

        rospy.sleep(1)
