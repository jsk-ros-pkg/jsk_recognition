import cv2
import math
import numpy as np
import rospy
import yaml


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
                              (track.rect.x + track.rect.width, track.rect.y + track.rect.height),
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
