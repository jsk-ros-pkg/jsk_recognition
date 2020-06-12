from __future__ import print_function

import cv2
import numpy as np
import itertools, pkg_resources, sys
from distutils.version import LooseVersion
if LooseVersion(pkg_resources.get_distribution("chainer").version) >= LooseVersion('7.0.0') and \
   sys.version_info.major == 2:
   print('''Please install chainer <= 7.0.0:

    sudo pip install chainer==6.7.0

c.f https://github.com/jsk-ros-pkg/jsk_recognition/pull/2485
''', file=sys.stderr)
   sys.exit(1)
if [p for p in list(itertools.chain(*[pkg_resources.find_distributions(_) for _ in sys.path])) if "cupy-" in p.project_name ] == []:
   print('''Please install CuPy

    sudo pip install cupy-cuda[your cuda version]
i.e.
    sudo pip install cupy-cuda91

''', file=sys.stderr)
   sys.exit(1)
import chainer

from jsk_recognition_utils.chainermodels.deep_sort_net\
    import DeepSortFeatureExtractor

from vis_bboxes import vis_bboxes
import deep_sort


def extract_image_patch(image, bbox, patch_shape):
    """Extract image patch from bounding box.
    copied from
    https://github.com/nwojke/deep_sort/blob/master/tools/generate_detections.py

    Parameters
    ----------
    image : ndarray
        The full image.
    bbox : array_like
        The bounding box in format (x, y, width, height).
    patch_shape : Optional[array_like]
        This parameter can be used to enforce a desired patch shape
        (height, width). First, the `bbox` is adapted to the aspect ratio
        of the patch shape, then it is clipped at the image boundaries.
        If None, the shape is computed from :arg:`bbox`.

    Returns
    -------
    ndarray | NoneType
        An image patch showing the :arg:`bbox`, optionally reshaped to
        :arg:`patch_shape`.
        Returns None if the bounding box is empty or fully outside of the image
        boundaries.

    """
    bbox = np.array(bbox)
    if patch_shape is not None:
        # correct aspect ratio to patch shape
        target_aspect = float(patch_shape[1]) / patch_shape[0]
        new_width = target_aspect * bbox[3]
        bbox[0] -= (new_width - bbox[2]) / 2
        bbox[2] = new_width

    # convert to top left, bottom right
    bbox[2:] += bbox[:2]
    bbox = bbox.astype(np.int)

    # clip at image boundaries
    bbox[:2] = np.maximum(0, bbox[:2])
    bbox[2:] = np.minimum(np.asarray(image.shape[:2][::-1]) - 1, bbox[2:])
    if np.any(bbox[:2] >= bbox[2:]):
        return None
    sx, sy, ex, ey = bbox
    image = image[sy:ey, sx:ex]
    image = cv2.resize(image, tuple(patch_shape[::-1]))
    return image


def encoder(image_encoder):

    def _encoder(image, boxes):
        image_shape = 128, 64, 3
        image_patches = []
        for box in boxes:
            patch = extract_image_patch(
                image, box, image_shape[:2])
            if patch is None:
                patch = np.random.uniform(
                    0., 255., image_shape).astype(np.uint8)
            image_patches.append(patch)
        image_patches = np.asarray(image_patches, 'f')
        image_patches = image_patches.transpose(0, 3, 1, 2)
        image_patches = image_encoder.xp.asarray(image_patches)
        with chainer.using_config('train', False):
            ret = image_encoder(image_patches)
        return chainer.cuda.to_cpu(ret.data)

    return _encoder


class DeepSortTracker(object):

    def __init__(self, gpu=-1,
                 pretrained_model=None,
                 nms_max_overlap=1.0,
                 max_cosine_distance=0.2,
                 budget=None):
        self.max_cosine_distance = max_cosine_distance
        self.nms_max_overlap = nms_max_overlap
        self.budget = budget

        # feature extractor
        self.gpu = gpu
        self.extractor = DeepSortFeatureExtractor()
        if pretrained_model is not None:
            chainer.serializers.load_npz(
                pretrained_model, self.extractor)
        if self.gpu >= 0:
            self.extractor = self.extractor.to_gpu()
        self.encoder = encoder(self.extractor)

        # variables for tracking objects
        self.n_tracked = 0  # number of tracked objects
        self.tracking_objects = {}
        self.tracker = None
        self.track_id_to_object_id = {}
        self.reset()

    def reset(self):
        self.track_id_to_object_id = {}
        self.tracking_objects = {}
        metric = deep_sort.deep_sort.nn_matching.NearestNeighborDistanceMetric(
            'cosine',
            matching_threshold=self.max_cosine_distance,
            budget=self.budget)
        self.tracker = deep_sort.deep_sort.tracker.Tracker(metric)

    def track(self, frame, bboxes, scores):
        # run non-maximam suppression.
        indices = deep_sort.application_util.preprocessing.non_max_suppression(
            bboxes, self.nms_max_overlap, scores)
        bboxes = bboxes[indices]
        scores = scores[indices]

        # generate detections.
        features = self.encoder(frame, np.array(bboxes))
        n_bbox = len(bboxes)
        detections = [
            deep_sort.deep_sort.detection.Detection(
                bboxes[i], scores[i], features[i]) for i in range(n_bbox)]

        # update tracker.
        self.tracker.predict()
        self.tracker.update(detections)

        for target_object in self.tracking_objects.values():
            target_object['out_of_frame'] = True

        # store results
        for track in self.tracker.tracks:
            if not track.is_confirmed() or track.time_since_update > 1:
                continue
            bbox = track.to_tlwh()

            if track.track_id in self.track_id_to_object_id:
                # update tracked object
                target_object = self.tracking_objects[
                    self.track_id_to_object_id[track.track_id]]
                target_object['out_of_frame'] = False
                target_object['bbox'] = bbox
            else:
                # detected for the first time
                object_id = self.n_tracked
                self.n_tracked += 1
                self.track_id_to_object_id[track.track_id] = object_id
                self.tracking_objects[object_id] = dict(
                    out_of_frame=False,
                    bbox=bbox)

    def visualize(self, frame, bboxes):
        vis_frame = frame.copy()
        for x1, y1, w, h in bboxes:
            x2, y2 = x1 + w, y1 + h
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
            cv2.rectangle(vis_frame,
                          (x1, y1), (x2, y2),
                          (255, 255, 255), 3)
        labels, bboxes = [], []
        for object_id, target_object in self.tracking_objects.items():
            if target_object['out_of_frame']:
                continue
            labels.append(object_id)
            bboxes.append(target_object['bbox'])
        vis_bboxes(vis_frame, bboxes, labels)
        return vis_frame
