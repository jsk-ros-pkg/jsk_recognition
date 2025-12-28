import numpy as np
from tf.transformations import quaternion_matrix as quaternion2matrix

from .cylinder import make_cylinder
from .sphere import make_sphere


def make_human_marker(head_radius=0.1,
                      body_radius=0.07,
                      height=1.8,
                      pos=[0.0, 0.0, 0.0],
                      q_xyzw=[0.0, 0.0, 0.0, 1.0],
                      color=(0.0, 0.0, 0.0, 1.0),
                      lifetime=0.25,
                      id=0,
                      frame_id='',
                      stamp=None):
    if height <= 0.0:
        raise ValueError('height should be greater than 0.0.')
    if head_radius <= 0.0:
        raise ValueError('head_radius should be greater than 0.0.')
    if height <= head_radius:
        raise ValueError('head_radius ({}) should be greater than height ({})'.
                         format(head_radius, height))

    body_height = height - head_radius * 2
    pos = np.array(pos, 'f')
    rotation = quaternion2matrix(q_xyzw)[:3, :3]
    head_pos = np.dot(rotation, [0, 0, height - head_radius]) + pos
    body_pos = np.dot(rotation, [0, 0, body_height / 2.0]) + pos

    head_marker = make_sphere(radius=head_radius,
                              pos=head_pos,
                              q_xyzw=q_xyzw,
                              color=color,
                              lifetime=lifetime,
                              id=id,
                              frame_id=frame_id,
                              stamp=stamp)
    body_marker = make_cylinder(
        radius=body_radius,
        height=body_height,
        pos=body_pos,
        q_xyzw=q_xyzw,
        color=color,
        lifetime=lifetime,
        id=id + 1,
        frame_id=frame_id,
        stamp=stamp)
    return [head_marker, body_marker]
