#!/usr/bin/python
PKG = 'cr_calibration' # this package name
import roslib; roslib.load_manifest(PKG)

import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import sensor_msgs.srv
import cv_bridge
import cv
import tf
import message_filters
import tf_conversions.posemath as pm
import PyKDL

import math
import os
import sys
import operator
import time
import numpy

class CrCalibration:
    def __init__(self, size, dim):

        self.chess_size = size
        self.dim = dim

        left_ns = rospy.remap_name('left')
        range_ns = rospy.remap_name('range')
        limg_sub = message_filters.Subscriber(left_ns + '/image_rect', sensor_msgs.msg.Image)
        rimg_sub = message_filters.Subscriber(range_ns + '/image_rect', sensor_msgs.msg.Image)
        linfo_sub = message_filters.Subscriber(left_ns + '/camera_info',sensor_msgs.msg.CameraInfo)
        rinfo_sub = message_filters.Subscriber(range_ns + '/camera_info', sensor_msgs.msg.CameraInfo)

        ts = message_filters.TimeSynchronizer([limg_sub, linfo_sub, rimg_sub, rinfo_sub], 16)
        ts.registerCallback(self.queue_cr)
        self.bridge = cv_bridge.CvBridge()

        self.frame_list = []
        self.static_pose = None
        self.result_list = []
        self.last_err = 0
 
    def calc_frame(self, lst):
        pos = PyKDL.Vector()
        ex = 0
        ey = 0
        ez = 0
        for f in lst:
            pos += f.p
            z, y, x = f.M.GetEulerZYX()
            ex += x
            ey += y
            ez += z

        size = len(lst)
        return PyKDL.Frame(PyKDL.Rotation.EulerZYX(ez/size, ey/size, ex/size), pos/size)

    def calc_distance(self, f1, f2):
        test = f1 * f2.Inverse()
        angle, axis = test.M.GetRotAngle()
        norm = test.p.Norm()
        return (norm + (angle / 10.0))

    def queue_cr(self, limg, linfo, rimg, rinfo):
        #
        lpose = self.find_checkerboard_pose(limg, linfo)
        rpose = self.find_checkerboard_pose(rimg, rinfo)
        if lpose == None or rpose == None:
            if lpose == None:
                rospy.loginfo("can't find CB on left camera image")
            if rpose == None:
                rospy.loginfo("can't find CB on range image")
            return False

        # calcurate pose
        lframe = pm.fromMsg(lpose.pose)
        rframe = pm.fromMsg(rpose.pose)
        frame = lframe * rframe.Inverse()

        # check pose euniqness
        for r in self.result_list:
            if self.calc_distance(r[0], lframe) < 0.075:
                return False

        # check pose_movement
        if len(self.frame_list) == 0:
            self.static_pose = lframe
            self.frame_list.append(frame)
            return False

        dist = self.calc_distance(self.static_pose, lframe)
        print(dist)
        if dist > 0.012:
            self.frame_list = []
            self.static_pose = None
            return False
        self.frame_list.append(frame)

        if len(self.frame_list) > 5:
            self.result_list.append([self.static_pose, self.calc_frame(self.frame_list)])
            self.frame_list = []
            self.static_pose = None
            # check resut list num
            ret = self.calc_frame([r[1] for r in self.result_list])
            err = 0.0
            for r in self.result_list:
                err += self.calc_distance(r[1], ret)

            qx, qy, qz, qw = ret.M.GetQuaternion()
            rospy.loginfo("%f %f %f %f %f %f %f / err = %f" % (ret.p.x(), ret.p.y(), ret.p.z(),
                                                               qx, qy, qz, qw, err/len(self.result_list)))
            self.last_err = err/len(self.result_list)

            # finish check
            if len(self.result_list) > 7 and self.last_err < 0.1:
                rospy.loginfo("Finished size = %d, err = %f" % (len(self.result_list), self.last_err))
                print("translation: [%f, %f, %f]\nrotation: [%f, %f, %f, %f]" % (ret.p.x(), ret.p.y(), ret.p.z(), qx, qy, qz, qw))
                print("(make-coords :pos #f(%f %f %f) :rot (quaternion2matrix #f(%f %f %f %f)))" % (1000*ret.p.x(), 1000*ret.p.y(), 1000*ret.p.z(), qw, qx, qy, qz))
                #print "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"cam_link_broadcaster\" args=\"%f %f %f %f %f %f %f link1 link2 30\" />" % (ret.p.x(), ret.p.y(), ret.p.z(), qw, qx, qy, qz)
                exit(-1)

            return True

        return True

    def detect(self, image):
        corners_x = self.chess_size[0]
        corners_y = self.chess_size[1]

        #Here, we'll actually call the openCV detector
        found, corners = cv.FindChessboardCorners(image, (corners_x, corners_y), cv.CV_CALIB_CB_ADAPTIVE_THRESH)

        if found:
            board_corners = (corners[0],
                             corners[corners_x  - 1],
                             corners[(corners_y - 1) * corners_x],
                             corners[len(corners) - 1])

            #find the perimeter of the checkerboard
            perimeter = 0.0
            for i in range(len(board_corners)):
                next = (i + 1) % 4
                xdiff = board_corners[i][0] - board_corners[next][0]
                ydiff = board_corners[i][1] - board_corners[next][1]
                perimeter += math.sqrt(xdiff * xdiff + ydiff * ydiff)

            #estimate the square size in pixels
            square_size = perimeter / ((corners_x - 1 + corners_y - 1) * 2)
            radius = int(square_size * 0.5 + 0.5)

            corners = cv.FindCornerSubPix(image, corners, (radius, radius), (-1, -1), (cv.CV_TERMCRIT_EPS | cv.CV_TERMCRIT_ITER, 30, 0.1))

            #uncomment to debug chessboard detection
            #cv.DrawChessboardCorners(image, (corners_x, corners_y), corners, 1)
            #cv.NamedWindow("image")
            #cv.ShowImage("image", image)
            #cv.WaitKey(600)

            #we'll also generate the object points if the user has specified spacing
            object_points = cv.CreateMat(3, corners_x * corners_y, cv.CV_32FC1)

            for y in range(corners_y):
                for x in range(corners_x):
                    cv.SetReal2D(object_points, 0, y*corners_x + x, x * self.dim)
                    cv.SetReal2D(object_points, 1, y*corners_x + x, y * self.dim)
                    cv.SetReal2D(object_points, 2, y*corners_x + x, 0.0)

            #not sure why opencv functions return non opencv compatible datatypes... but they do so we'll convert
            corners_cv = cv.CreateMat(2, corners_x * corners_y, cv.CV_32FC1)
            for i in range(corners_x * corners_y):
                cv.SetReal2D(corners_cv, 0, i, corners[i][0])
                cv.SetReal2D(corners_cv, 1, i, corners[i][1])

            return (corners_cv, object_points)
        else:
            #cv.NamedWindow("image_scaled")
            #cv.ShowImage("image_scaled", image_scaled)
            #cv.WaitKey(600)
            rospy.logwarn("Didn't find checkerboard")
            return (None, None)
        return

    def intrinsic_matrix_from_info(self, cam_info):
        intrinsic_matrix = cv.CreateMat(3, 3, cv.CV_32FC1)

        #Because we only want the upper 3x3 (normal) portion of the rectified intrinsic matrix
        for i in range(0, 3):
            for j in range(0, 3):
                intrinsic_matrix[i, j] = cam_info.P[4*i+j]
        return intrinsic_matrix

    def find_checkerboard_pose(self, ros_image, cam_info):
    #we need to convert the ros image to an opencv image
        try:
            image = self.bridge.imgmsg_to_cv(ros_image, "mono8")
        except CvBridgeError as e:
            rospy.logerror("Error importing image %s" % e)
            return

        corners, model = self.detect(image)

        if corners == None or model == None:
            return None
        else:
            #find the pose of the checkerboard
            rot = cv.CreateMat(3, 1, cv.CV_32FC1)
            trans = cv.CreateMat(3, 1, cv.CV_32FC1)
            kc = cv.CreateMat(1, 4, cv.CV_32FC1)
            cv.Set(kc, 0.0)
            intrinsic_matrix = self.intrinsic_matrix_from_info(cam_info)

            cv.FindExtrinsicCameraParams2(model, corners, intrinsic_matrix, kc, rot, trans)

            #We want to build a transform now, but first we have to convert the
            #rotation vector we get back from OpenCV into a rotation matrix
            rot_mat = cv.CreateMat(3, 3, cv.CV_32FC1)
            cv.Rodrigues2(rot, rot_mat)

            #Now we need to convert this rotation matrix into a quaternion
            #This can be done better in never versions of opencv, but we need to be boxturtle compatible
            numpy_mat = numpy.fromstring(rot_mat.tostring(), dtype = numpy.float32).reshape((3,3))

            #of course, tf expects all matricies passed to it to be 4x4... so we'll build a full matrix here
            full_pose = numpy.zeros((4, 4))

            #first, copy in the rotation matrix
            full_pose[0:3, 0:3] = numpy_mat[0:3, 0:3]

            #next, we'll copy in the translation
            full_pose[0:3, 3] = [trans[i, 0] for i in range(3)]

            #and make sure to add a 1 in the lower right corner
            full_pose[3][3] = 1.0

            rospy.logdebug("%s" % numpy_mat)
            rospy.logdebug("%s" % full_pose)

            tf_trans = tf.transformations.translation_from_matrix(full_pose)
            tf_rot = tf.transformations.quaternion_from_matrix(full_pose)

            #and now we'll actually build our pose stamped
            board_pose = geometry_msgs.msg.PoseStamped()
            board_pose.header = ros_image.header
            board_pose.pose.position.x = tf_trans[0]
            board_pose.pose.position.y = tf_trans[1]
            board_pose.pose.position.z = tf_trans[2]
            board_pose.pose.orientation.x = tf_rot[0]
            board_pose.pose.orientation.y = tf_rot[1]
            board_pose.pose.orientation.z = tf_rot[2]
            board_pose.pose.orientation.w = tf_rot[3]
            rospy.logdebug("%s" % board_pose)

            #we'll publish the pose so we can display it in rviz
            # self.pose_pub.publish(board_pose)
            return board_pose

def main():
    from optparse import OptionParser
    rospy.init_node('cr_calibrator')
    parser = OptionParser()
    parser.add_option("-s", "--size", default="8x6", help="specify chessboard size as nxm [default: %default]")
    parser.add_option("-q", "--square", default=".108", help="specify chessboard square size in meters [default: %default]")

    options, args = parser.parse_args()
    size = tuple([int(c) for c in options.size.split('x')])
    dim = float(options.square)
    node = CrCalibration(size, dim)

    rospy.spin()

if __name__ == "__main__":
    main()

