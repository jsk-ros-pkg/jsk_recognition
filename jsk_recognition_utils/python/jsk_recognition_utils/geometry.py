import numpy as np
from tf.transformations import unit_vector as normalize_vector


def outer_product_matrix(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def cross_product(a, b):
    return np.dot(outer_product_matrix(a), b)


def rotation_matrix_from_axis(
        first_axis=(1, 0, 0), second_axis=(0, 1, 0), axes='xy'):
    if axes not in ['xy', 'yx', 'xz', 'zx', 'yz', 'zy']:
        raise ValueError("Valid axes are 'xy', 'yx', 'xz', 'zx', 'yz', 'zy'.")
    e1 = normalize_vector(first_axis)
    if np.linalg.norm(e1, ord=2) == 0:
        raise ValueError(
            'Invalid first_axis value. '
            'Norm of axis should be greater than 0.0. Get first_axis: ({})'.
            format(first_axis))
    e2 = normalize_vector(second_axis - np.dot(second_axis, e1) * e1)
    if np.linalg.norm(e2, ord=2) == 0:
        raise ValueError(
            'Invalid second_axis({}) with respect to first_axis({}).'
            .format(first_axis, second_axis))
    if axes in ['xy', 'zx', 'yz']:
        third_axis = cross_product(e1, e2)
    else:
        third_axis = cross_product(e2, e1)
    e3 = normalize_vector(
        third_axis - np.dot(third_axis, e1) * e1 - np.dot(third_axis, e2) * e2)
    first_index = ord(axes[0]) - ord('x')
    second_index = ord(axes[1]) - ord('x')
    third_index = ((first_index + 1) ^ (second_index + 1)) - 1
    indices = [first_index, second_index, third_index]
    return np.vstack([e1, e2, e3])[np.argsort(indices)].T


def get_overlap_of_aabb(box1, box2, return_volumes=False):
    x11, y11, z11, x12, y12, z12 = box1
    dim_x1 = x12 - x11
    dim_y1 = y12 - y11
    dim_z1 = z12 - z11

    x21, y21, z21, x22, y22, z22 = box2
    dim_x2 = x22 - x21
    dim_y2 = y22 - y21
    dim_z2 = z22 - z21

    if ((x11 <= x22 and x12 >= x21) and
            (y11 <= y22 and y12 >= y21) and
            (z11 <= z22 and z12 >= z21)):
        # has intersect
        dim_x3 = min(x12, x22) - max(x11, x21)
        dim_y3 = min(y12, y22) - max(y11, y21)
        dim_z3 = min(z12, z22) - max(z11, z21)
    else:
        dim_x3 = dim_y3 = dim_z3 = 0

    intersect = dim_x3 * dim_y3 * dim_z3
    union = (dim_x1 * dim_y1 * dim_z1) + (dim_x2 * dim_y2 * dim_z2) - intersect
    iu = 1. * intersect / union
    if return_volumes:
        return iu, intersect, union
    else:
        return iu


def pairwise_distances(a, b):
    """Compute pair-wise distance between points in `a` and `b`.

    Parameters
    ----------
    a : array_like
        An NxM matrix of N samples of dimensionality M.
    b : array_like
        An LxM matrix of L samples of dimensionality M.

    Returns
    -------
    ndarray
        Returns a matrix of size len(a), len(b) such that eleement (i, j)
        contains the distance between `a[i]` and `b[j]`.
    """
    a, b = np.asarray(a), np.asarray(b)
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)))
    a2, b2 = np.square(a).sum(axis=1), np.square(b).sum(axis=1)
    r2 = -2. * np.dot(a, b.T) + a2[:, None] + b2[None, :]
    r2 = np.clip(r2, 0., float(np.inf))
    return np.sqrt(r2)


def euclidean_distances(a, b, a_indices=None, b_indices=None):
    if a is None:
        a_indices = np.arange(len(a))
    if b is None:
        b_indices = np.arange(len(b))
    return pairwise_distances(a[a_indices], b[b_indices])
