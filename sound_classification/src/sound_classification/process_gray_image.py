import cv2
import matplotlib.cm as cm
import numpy as np


def spectral_subtract(img, noise):
    spectrogram_subtracted = img.transpose() - noise
    # Spectral subtraction method
    spectrogram_subtracted = np.where(spectrogram_subtracted > 0,
                                      spectrogram_subtracted,
                                      noise * 0.01)
    spectrogram_subtracted = spectrogram_subtracted.transpose()
    return spectrogram_subtracted


def smooth_gray_image(raw_img):
    """
    Blur to gray image
    input:  cv2 32FC1 image
    output: cv2 32FC1 image
    """
    return cv2.blur(raw_img, (5, 5))


def normalize_gray_image(img):
    """
    Convert input gray image to 8FC1 gray image.
    At this time, each pixel is normalized between 0 ~ 255.
    input:  cv2 image
    output: cv2 image
    """
    dtype = img.dtype
    _max = img.max()
    _min = img.min()
    ret = (img - _min).astype(np.float64) / (_max - _min) * 255.0
    return ret.astype(dtype)


def img_jet(img):
    """
    Convert input to jet image if input is mono image.
    input : cv2 8UC1 image
    output: cv2 8UC3 image
    """
    if len(img.shape) == 2:
        normalized_img = img / 255.0
        jet = np.array(cm.jet(1 - normalized_img)[:, :, :3] * 255, np.uint8)
    else:
        jet = img
    return jet
