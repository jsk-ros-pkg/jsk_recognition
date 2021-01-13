from __future__ import print_function

import sys

from jsk_recognition_utils import color
from jsk_recognition_utils import conversations
from jsk_recognition_utils import feature
from jsk_recognition_utils import mask
from jsk_recognition_utils import visualize
from jsk_recognition_utils import geometry


try:
    import chainer  # NOQA
    _chainer_available = True
except ImportError:
    _chainer_available = False

try:
    import chainercv  # NOQA
    _chainercv_available = True
except ImportError:
    _chainercv_available = False

try:
    import fcn  # NOQA
    _fcn_available = True
except ImportError:
    _fcn_available = False

if _chainer_available and _chainercv_available and _fcn_available:
    from jsk_recognition_utils import chainermodels  # NOQA
else:
    _depends = []
    if not _chainer_available:
        _depends.append('chainer\\<7.0.0')
    if not _chainercv_available:
        _depends.append('chainercv')
    if not _fcn_available:
        _depends.append('fcn')
    print('''
Please install {0}
to import jsk_recognition_utils.chainermodels.

    sudo pip install {1}
'''.format(', '.join(_depends), ' '.join(_depends)), file=sys.stderr)

if _chainer_available and _chainercv_available:
    from jsk_recognition_utils import datasets  # NOQA
else:
    _depends = []
    if not _chainer_available:
        _depends.append('chainer\\<7.0.0')
    if not _chainercv_available:
        _depends.append('chainercv')
    print('''
Please install {0}
to import jsk_recognition_utils.datasets.

    sudo pip install {1}
'''.format(', '.join(_depends), ' '.join(_depends)), file=sys.stderr)


bounding_box_msg_to_aabb = conversations.bounding_box_msg_to_aabb
rects_msg_to_ndarray = conversations.rects_msg_to_ndarray

BagOfFeatures = feature.BagOfFeatures
decompose_descriptors_with_label = feature.decompose_descriptors_with_label

bounding_rect_of_mask = mask.bounding_rect_of_mask
descent_closing = mask.descent_closing

centerize = visualize.centerize
colorize_cluster_indices = visualize.colorize_cluster_indices
get_tile_image = visualize.get_tile_image

get_overlap_of_aabb = geometry.get_overlap_of_aabb
