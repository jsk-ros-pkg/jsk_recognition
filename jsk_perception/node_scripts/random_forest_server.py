#!/usr/bin/env python

try:
    from ml_classifiers.srv import *
except:
    import roslib;roslib.load_manifest("ml_classifiers")
    from ml_classifiers.srv import *

import rospy
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.ensemble import ExtraTreesClassifier
from sklearn.externals import joblib


class RandomForestServer:
    def __init__(self, clf):
        self.clf = clf
        s = rospy.Service('predict', ClassifyData, self.classifyData)

    @classmethod
    def initWithData(cls, data_x, data_y):
        if len(data_x) != len(data_y):
            rospy.logerr("Lenght of datas are different")
            exit()
        rospy.loginfo("InitWithData please wait..")
        clf = RandomForestClassifier(
            n_estimators=250, max_features=2, max_depth=29,
            min_samples_split=2, random_state=0)
        clf.fit(data_x, data_y)
        return cls(clf)

    @classmethod
    def initWithFileModel(cls, filename):
        rospy.loginfo("InitWithFileModel with%s please wait.."%filename)
        clf = joblib.load(filename)
        return cls(clf)

    #Return predict result
    def classifyData(self, req):
        ret = []
        for data in req.data:
            print(data)
            ret.append(" ".join([
                str(predict_data)
                for predict_data in self.clf.predict([data.point])]))
            rospy.loginfo("req : " + str(data.point) +  "-> answer : " + str(ret))
        return ClassifyDataResponse(ret)

    #Run random forest
    def run(self):
        rospy.loginfo("RandomForestServer is running!")
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node('random_forest_cloth_classifier')

    try:
        train_file = rospy.get_param('~random_forest_train_file')
    except KeyError:
        rospy.logerr("Train File is not Set. Set train_data file or tree model file as ~random_forest_train_file.")
        exit()

    if train_file.endswith("pkl"):
        node = RandomForestServer.initWithFileModel(train_file)
    else:
        try:
            class_file = rospy.get_param('~random_forest_train_class_file')

            data_x = []
            data_y = []
            for l in open(train_file).readlines():
                float_strings = l.split(",");
                data_x.append(map(lambda x: float(x), float_strings))

            for l in open(class_file).readlines():
                data_y.append(float(l))

            #build servece server
            node = RandomForestServer.initWithData(np.array(data_x), np.array(data_y))

        except KeyError:
            rospy.logerr("Train Class File is not Set. Set train_data file or tree model file.")
            rospy.logerr("Or Did you expect  Extension to be pkl?.")
            exit()


    #run
    node.run()
