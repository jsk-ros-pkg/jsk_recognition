#!/usr/bin/env python

# Mainly copied from
# https://gist.github.com/rossbar/ebb282c3b73c41c1404123de6cea4771
# https://answers.ros.org/question/236976/how-to-publish-camera_info-manually/

import rospy
import yaml
from sensor_msgs.msg import CameraInfo, Image


class AddCameraInfo(object):
    def __init__(self):
        self.publisher = rospy.Publisher(
            "~output", CameraInfo, queue_size=10)
        rospy.Subscriber(
            '~input', Image, self.cb, queue_size=10)
        self.yaml_filename = rospy.get_param(
            '~yaml_filename', '~/.ros/camera_info/camera.yaml')

    def yaml_to_CameraInfo(self, yaml_fname):
        """
        Parse a yaml file containing camera calibration data (as produced by
        rosrun camera_calibration cameracalibrator.py) into a
        sensor_msgs/CameraInfo msg.

        Parameters
        ----------
        yaml_fname : str
            Path to yaml file containing camera calibration data
        Returns
        -------
        camera_info_msg : sensor_msgs.msg.CameraInfo
            A sensor_msgs.msg.CameraInfo message containing the camera calibration
            data
        """
        # Load data from file
        with open(yaml_fname, "r") as file_handle:
            calib_data = yaml.load(file_handle)
        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["camera_model"]
        return camera_info_msg

    def cb(self, msg):
        camera_info = self.yaml_to_CameraInfo(self.yaml_filename)
        camera_info.header = msg.header
        self.publisher.publish(camera_info)


if __name__ == "__main__":
    # Initialize publisher node
    rospy.init_node("add_camera_info", anonymous=True)
    AddCameraInfo()
    # Run publisher
    while not rospy.is_shutdown():
        rospy.spin()
