import cv2
import math
import numpy as np
import rospy
import yaml
import PyKDL

from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray


def transform_panorama_point(x,
                             y,
                             image_height,
                             image_width,
                             theta_min,
                             theta_max,
                             phi_min,
                             phi_max):
    phi = phi_max + 1.0 * (phi_min - phi_max) * x / image_width
    theta = theta_min + 1.0 * (theta_max - theta_min) * y / image_height
    return (theta, phi)


def calc_spherical_point(theta, phi, r):
    return (r * math.sin(theta) * math.cos(phi),
            r * math.sin(theta) * math.sin(phi),
            r * math.cos(theta))


def visualize_tracks(input_frame, tracks_msg):

    frame = input_frame
    for track in tracks_msg.tracks:
        H = (track.track_id * 71) % 256
        S = 255
        V = 255
        bgr = cv2.cvtColor(
            np.array([[[H, S, V]]], dtype=np.uint8), cv2.COLOR_HSV2BGR)[0][0]
        color = (int(bgr[2]), int(bgr[1]), int(bgr[0]))
        frame = cv2.rectangle(frame,
                              (track.rect.x, track.rect.y),
                              (track.rect.x + track.rect.width,
                               track.rect.y + track.rect.height),
                              color,
                              3
                              )
    return frame


def load_label_names():
    label_names = rospy.get_param("~label_names")
    if isinstance(label_names, str):
        with open(label_names, "r") as f:
            label_names = tuple(yaml.load(f, yaml.Loader))
    return label_names


def transform_rect_in_panorama_to_bounding_box(
        list_rect,
        list_label_name,
        list_label_id,
        list_value,
        frame_fixed_to_panorama,
        frame_id_fixed,
        timestamp,
        dimensions_labels,
        image_width,
        image_height,
        fov_theta_min,
        fov_theta_max,
        fov_phi_min,
        fov_phi_max):

    msg_bbox_array = BoundingBoxArray()
    msg_bbox_array.header.frame_id = frame_id_fixed
    msg_bbox_array.header.stamp = timestamp

    for rect, label_name, label_id, value \
            in zip(list_rect, list_label_name, list_label_id, list_value):

        if label_name not in dimensions_labels:
            # For melodic or newer
            # https://wiki.ros.org/rospy/Overview/Logging#Logging_Periodically
            try:
                rospy.logwarn_throttle_identical(
                    600, 'height for label "{}" (id:{}) is not specified'.format(label_name,label_id))
            # For kinetic or older (logwarn_throttle_identical is not defined)
            except AttributeError:
                rospy.logwarn('height for label "{}" (id:{}) is not specified'.format(label_name,label_id))
            continue

        (theta_a, phi_a) = transform_panorama_point(
            rect.x,
            rect.y,
            image_height,
            image_width,
            fov_theta_min,
            fov_theta_max,
            fov_phi_min,
            fov_phi_max
        )
        (theta_b, phi_b) = transform_panorama_point(
            rect.x + rect.width,
            rect.y + rect.height,
            image_height,
            image_width,
            fov_theta_min,
            fov_theta_max,
            fov_phi_min,
            fov_phi_max
        )
        theta = (theta_a + theta_b) / 2.0
        phi = (phi_a + phi_b) / 2.0
        distance = (dimensions_labels[label_name][2] / 2.0) \
            / math.tan((theta_b - theta_a) / 2.0)

        (x, y, z) = calc_spherical_point(theta, phi, distance)
        position_fixedbased = frame_fixed_to_panorama * PyKDL.Vector(x, y, z)

        msg_bbox = BoundingBox()
        msg_bbox.header = msg_bbox_array.header
        msg_bbox.pose.position.x = position_fixedbased[0]
        msg_bbox.pose.position.y = position_fixedbased[1]
        msg_bbox.pose.position.z = position_fixedbased[2]
        msg_bbox.pose.orientation.w = 1.0
        msg_bbox.dimensions.x = dimensions_labels[label_name][0]
        msg_bbox.dimensions.y = dimensions_labels[label_name][1]
        msg_bbox.dimensions.z = dimensions_labels[label_name][2]
        msg_bbox.label = label_id
        msg_bbox.value = value

        msg_bbox_array.boxes.append(msg_bbox)

    return msg_bbox_array
