#!/usr/bin/env python

import numpy as np

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf.transformations

from geometry_msgs.msg import Pose, PoseArray
from jsk_topic_tools import ConnectionBasedTransport
from sensor_msgs.msg import PointCloud2
from sklearn.decomposition import PCA

class DetectGraspablePosesPcabase(ConnectionBasedTransport):

    def __init__(self):

        rospy.logwarn("This node is experiential one.")

        super(DetectGraspablePosesPcabase, self).__init__()

        self.direction = rospy.get_param('~direction', 'x')
        self.hand_size = rospy.get_param('~hand_size', 0.13) # maximum hand width
        self.interval_m = rospy.get_param('~interval_m', '0.04') # interval of possible grasp

        self.pub_target_poses = self.advertise("~output/can_grasp_poses", PoseArray, queue_size=1)

    def subscribe(self):
        rospy.Subscriber('~input', PointCloud2, self.callback)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, point_cloud):

        points = np.array(list(pc2.read_points(point_cloud, skip_nans=True)))

        interval_m = self.interval_m

        if self.direction == 'z':
            points_surface = points[:, [0, 1]] # choose x, y
        elif self.direction == 'x':
            points_surface = points[:, [1, 2]] # choose y, z
        else:
            rospy.logwarn("direction should be x or z")

        pca = PCA(n_components=2)
        pca.fit(points_surface)
        new_points = pca.transform(points_surface)
        new_points = new_points[np.argsort(new_points[:, 0])]

        min_x = new_points[0, 0]
        max_x = new_points[-1, 0]

        discrete_xy_list = []

        while min_x <= max_x:
            candidate = new_points[np.where(new_points[:, 0] < (min_x + interval_m))]
            new_points = new_points[np.where(new_points[:, 0] >= (min_x + interval_m))]
            if len(candidate) > 0:
                mean_x = np.mean(candidate, axis=0)[0]
                mean_y = np.mean(candidate, axis=0)[1]
                range_y = np.max(candidate[:, 1]) - np.min(candidate[:, 1])
                discrete_xy_list.append([mean_x, mean_y, range_y])
            min_x += interval_m

        discrete_xy_list = np.array(discrete_xy_list)
        discrete_xy_list = discrete_xy_list[np.where(discrete_xy_list[:, 2] < self.hand_size)]
        discrete_xy_list = discrete_xy_list[np.argsort(np.absolute(discrete_xy_list[:, 0]))]
        grasp_xy_list = pca.inverse_transform(discrete_xy_list[:, [0, 1]])

        search_range_m = interval_m

        grasp_position_list = []

        for grasp_xy in grasp_xy_list:
            if self.direction == 'z':
                points_depth = points[np.where((points[:, 0] > grasp_xy[0] - search_range_m) &
                                               (points[:, 0] < grasp_xy[0] + search_range_m) &
                                               (points[:, 1] > grasp_xy[1] - search_range_m) &
                                               (points[:, 1] < grasp_xy[1] + search_range_m))]
                points_depth = points_depth[:, 2]
            elif self.direction == 'x':
                points_depth = points[np.where((points[:, 1] > grasp_xy[0] - search_range_m) &
                                               (points[:, 1] < grasp_xy[0] + search_range_m) &
                                               (points[:, 2] > grasp_xy[1] - search_range_m) &
                                               (points[:, 2] < grasp_xy[1] + search_range_m))]
                points_depth = points_depth[:, 0]
            else:
                rospy.logwarn("direction should be x or z")
            if len(points_depth) > 0:
                grasp_position = np.append(grasp_xy, np.mean(points_depth))
            else:
                rospy.logwarn("it happens that something wrong")
            grasp_position_list.append(list(grasp_position))

        pub_msg = PoseArray()
        pub_msg.header = point_cloud.header

        trans_matrix = tf.transformations.identity_matrix()
        if self.direction == 'z':
            trans_matrix[0, 0] = 0
            trans_matrix[1, 0] = 0
            trans_matrix[2, 0] = -1
            trans_matrix[0, 1] = -1 * pca.components_[0, 1]
            trans_matrix[1, 1] = pca.components_[0, 0]
            trans_matrix[0, 2] = pca.components_[0, 0]
            trans_matrix[1, 2] = pca.components_[0, 1]
            trans_matrix[2, 2] = 0

        elif self.direction == 'x':
            # this if else is set so that z value of grasp poses' y axies become positive.
            if pca.components_[0, 0] > 0:
                trans_matrix[1, 1] = pca.components_[0, 0]
                trans_matrix[2, 1] = pca.components_[0, 1]
                trans_matrix[1, 2] = -1 * pca.components_[0, 1]
                trans_matrix[2, 2] = pca.components_[0, 0]
            else:
                trans_matrix[1, 1] = -1 * pca.components_[0, 0]
                trans_matrix[2, 1] = -1 * pca.components_[0, 1]
                trans_matrix[1, 2] = pca.components_[0, 1]
                trans_matrix[2, 2] = -1 * pca.components_[0, 0]
        else:
            rospy.logwarn("direction should be x or z")

        quaternion = tf.transformations.quaternion_from_matrix(trans_matrix)

        for grasp_position in grasp_position_list:
            pose = Pose()
            if self.direction == 'z':
                pose.position.x = grasp_position[0]
                pose.position.y = grasp_position[1]
                pose.position.z = grasp_position[2]
            elif self.direction == 'x':
                pose.position.x = grasp_position[2]
                pose.position.y = grasp_position[0]
                pose.position.z = grasp_position[1]
            else:
                rospy.logwarn("direction should x or z")
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            pub_msg.poses.append(pose)

        self.pub_target_poses.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node('detect_graspable_poses_pcabase')
    DetectGraspablePosesPcabase()
    rospy.spin()
