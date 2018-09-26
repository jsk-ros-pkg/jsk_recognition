#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import matplotlib.pyplot as plt

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf.transformations

from geometry_msgs.msg import Pose, PoseArray
from jsk_topic_tools import ConnectionBasedTransport
from sensor_msgs.msg import PointCloud2
from sklearn.decomposition import PCA

class DetectGraspablePosesPcabase(ConnectionBasedTransport):

    def __init__(self):

        rospy.init_node('detect_graspable_poses_pcabase')

        rospy.logwarn("This node is experiential one.")

        super(DetectGraspablePosesPcabase, self).__init__()

        self.direction = rospy.get_param('~direction', 'x')
        self.hand_size = rospy.get_param('~hand_size', 0.13) # maximum hand width
        self.interval_m = rospy.get_param('~interval_m', '0.04') # interval of possible grasp

        self.pub_targetPoses = self.advertise("~output/can_grasp_poses", PoseArray, queue_size=1)

        rospy.spin()

    def subscribe(self):
        rospy.Subscriber('~input', PointCloud2, self.callback)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, point_cloud):

        points = np.array(list(pc2.read_points(point_cloud, skip_nans=True)))

        interval_m = self.interval_m

        if self.direction == 'z':
            points_surface = points[:,[0,1]] # choose x, y
        elif self.direction == 'x':
            points_surface = points[:,[1,2]] # choose y, z
        else:
            rospy.logwarn("direction should x or z")

        pca = PCA(n_components=2)
        pca.fit(points_surface)
        newPoints = pca.transform(points_surface)
        newPoints = newPoints[np.argsort(newPoints[:,0])]

        minX = newPoints[0,0]
        maxX = newPoints[-1,0]

        discreteXYList = []

        while minX <= maxX:
            candidate = newPoints[np.where(newPoints[:,0] < (minX + interval_m))]
            newPoints = newPoints[np.where(newPoints[:,0] >= (minX + interval_m))]
            if len(candidate) > 0:
                meanX = np.mean(candidate, axis=0)[0]
                meanY = np.mean(candidate, axis=0)[1]
                rangeY = np.max(candidate[:,1]) - np.min(candidate[:,1])
                discreteXYList.append([meanX, meanY, rangeY])
            minX += interval_m

        discreteXYList = np.array(discreteXYList)
        discreteXYList = discreteXYList[np.where(discreteXYList[:,2] < self.hand_size)]
        discreteXYList = discreteXYList[np.argsort(np.absolute(discreteXYList[:,0]))]

        graspXYList = pca.inverse_transform(discreteXYList[:,[0,1]])

        searchRange_m = interval_m

        graspPositionList = []

        for graspXY in graspXYList:
            if self.direction == 'z':
                points_depth = points[np.where((points[:,0] > graspXY[0] - searchRange_m) &
                                               (points[:,0] < graspXY[0] + searchRange_m) &
                                               (points[:,1] > graspXY[1] - searchRange_m) &
                                               (points[:,1] < graspXY[1] + searchRange_m))]
                points_depth = points_depth[:,2]
            elif self.direction == 'x':
                points_depth = points[np.where((points[:,1] > graspXY[0] - searchRange_m) &
                                               (points[:,1] < graspXY[0] + searchRange_m) &
                                               (points[:,2] > graspXY[1] - searchRange_m) &
                                               (points[:,2] < graspXY[1] + searchRange_m))]
                points_depth = points_depth[:,0]
            else:
                rospy.logwarn("direction should x or z")
            if len(points_depth) > 0:
                graspPosition = np.append(graspXY, np.mean(points_depth))
            else:
                rospy.logwarn("it happens that something wrong")
            graspPositionList.append(list(graspPosition))

        pubMsg = PoseArray()
        pubMsg.header = point_cloud.header

        transMatrix = tf.transformations.identity_matrix()
        if self.direction == 'z':
            transMatrix[0,0] = 0
            transMatrix[1,0] = 0
            transMatrix[2,0] = -1
            transMatrix[0,1] = -1 * pca.components_[0,1]
            transMatrix[1,1] = pca.components_[0,0]
            transMatrix[0,2] = pca.components_[0,0]
            transMatrix[1,2] = pca.components_[0,1]
            transMatrix[2,2] = 0

        elif self.direction == 'x':
            transMatrix[1,1] = pca.components_[0,0]
            transMatrix[2,1] = pca.components_[0,1]
            transMatrix[1,2] = pca.components_[0,1]
            transMatrix[2,2] = -1 * pca.components_[0,0]

        else:
            rospy.logwarn("direction should x or z")

        quaternion = tf.transformations.quaternion_from_matrix(transMatrix)

        for graspPosition in graspPositionList:
            pose = Pose()
            if self.direction == 'z':
                pose.position.x = graspPosition[0]
                pose.position.y = graspPosition[1]
                pose.position.z = graspPosition[2]
            elif self.direction == 'x':
                pose.position.x = graspPosition[2]
                pose.position.y = graspPosition[0]
                pose.position.z = graspPosition[1]
            else:
                rospy.logwarn("direction should x or z")
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            pubMsg.poses.append(pose)

        self.pub_targetPoses.publish(pubMsg)

def main():
    try:
        DetectGraspablePosesPcabase()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
