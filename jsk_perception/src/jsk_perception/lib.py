import math


def transformPanoramaPoint(x,
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


def calcSphericalPoint(theta, phi, r):
    return (r * math.sin(theta) * math.cos(phi),
            r * math.sin(theta) * math.sin(phi),
            r * math.cos(theta))
