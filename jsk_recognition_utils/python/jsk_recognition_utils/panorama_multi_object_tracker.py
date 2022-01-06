from __future__ import print_function
import sys
if sys.version_info.major < 3:
    print('Please use python3 to import panorama_multi_object_tracker', file=sys.stderr)
    raise ImportError

from motpy.core import Box, Detection, Track, Vector, setup_logger
from motpy.metrics import angular_similarity
from motpy.model import Model, ModelPreset
from motpy.tracker import SingleObjectTracker, BaseMatchingFunction
from motpy.tracker import KalmanTracker, SimpleTracker
from motpy.tracker import EPS
from motpy.tracker import _sequence_has_none
from motpy.tracker import DEFAULT_MODEL_SPEC
import numpy as np
import scipy
from typing import (Any, Callable, Dict, List, Optional, Sequence, Tuple, Type, Union)
import time

logger = setup_logger(__name__)


def calculate_iou_panorama(bboxes1, bboxes2, image_width, dim: int = 2):
    """ expected bboxes size: (-1, 2*dim) """
    bboxes1 = np.array(bboxes1).reshape((-1, dim * 2))
    bboxes2 = np.array(bboxes2).reshape((-1, dim * 2))

    coords_b1 = np.split(bboxes1, 2 * dim, axis=1)
    coords_b2 = np.split(bboxes2, 2 * dim, axis=1)

    coords = np.zeros(shape=(2, dim, bboxes1.shape[0], bboxes2.shape[0]))
    volume_inter, volume_b1, volume_b2 = 1.0, 1.0, 1.0
    for d in range(dim):
        if d == 0: # x
            top_left = np.maximum(coords_b1[d], np.transpose(coords_b2[d]))  # x_min, y_min
            bottom_right = np.minimum(coords_b1[d + dim], np.transpose(coords_b2[d + dim]))  # x_max, y_max
            temp_vol_inter = np.maximum(bottom_right - top_left, 0)

            top_left = np.maximum(coords_b1[d], np.transpose(coords_b2[d] + image_width))  # x_min, y_min
            bottom_right = np.minimum(coords_b1[d + dim], np.transpose(coords_b2[d + dim] + image_width))  # x_max, y_max
            temp_vol_inter_plus = np.maximum(bottom_right - top_left, 0)

            top_left = np.maximum(coords_b1[d], np.transpose(coords_b2[d] - image_width))  # x_min, y_min
            bottom_right = np.minimum(coords_b1[d + dim], np.transpose(coords_b2[d + dim] - image_width))  # x_max, y_max
            temp_vol_inter_minus = np.maximum(bottom_right - top_left, 0)

            volume_inter *= np.maximum(np.maximum(temp_vol_inter_plus, temp_vol_inter_minus), temp_vol_inter)
            volume_b1 *= coords_b1[d + dim] - coords_b1[d]
            volume_b2 *= coords_b2[d + dim] - coords_b2[d]

        else: # y
            top_left = np.maximum(coords_b1[d], np.transpose(coords_b2[d]))  # top-left
            bottom_right = np.minimum(coords_b1[d + dim], np.transpose(coords_b2[d + dim]))  # bottom-right

            volume_inter *= np.maximum(bottom_right - top_left, 0)
            volume_b1 *= coords_b1[d + dim] - coords_b1[d]
            volume_b2 *= coords_b2[d + dim] - coords_b2[d]

    iou = volume_inter / (np.clip(volume_b1 + np.transpose(volume_b2) - volume_inter, a_min=0, a_max=None) + EPS)
    #print('length of bbox tracker: {}'.format(bboxes1.shape[0]))
    #print('length of bbox detections: {}'.format(bboxes2.shape[0]))
    #print('iou: {}'.format(iou))
    return iou


def cost_matrix_iou_feature_for_panorama(trackers: Sequence[SingleObjectTracker],
                                         detections: Sequence[Detection],
                                         image_width,
                                         feature_similarity_fn=angular_similarity,
                                         feature_similarity_beta: float = None) -> Tuple[np.ndarray, np.ndarray]:

    # boxes
    b1 = np.array([t.box() for t in trackers])
    b2 = np.array([d.box for d in detections])

    # box iou
    inferred_dim = int(len(b1[0]) / 2)
    iou_mat = calculate_iou_panorama(b1, b2, image_width, dim=inferred_dim)

    # feature similarity
    if feature_similarity_beta is not None:
        # get features
        f1 = [t.feature for t in trackers]
        f2 = [d.feature for d in detections]

        if _sequence_has_none(f1) or _sequence_has_none(f2):
            # fallback to pure IOU due to missing features
            apt_mat = iou_mat
        else:
            sim_mat = feature_similarity_fn(f1, f2)
            sim_mat = feature_similarity_beta + \
                (1 - feature_similarity_beta) * sim_mat

            # combined aptitude
            apt_mat = np.multiply(iou_mat, sim_mat)
    else:
        apt_mat = iou_mat

    cost_mat = -1.0 * apt_mat
    return cost_mat, iou_mat


def match_by_cost_matrix_for_panorama(trackers: Sequence[SingleObjectTracker],
                                      detections: Sequence[Detection],
                                      image_width,
                                      min_iou: float = 0.1,
                                      multi_match_min_iou: float = 1. + EPS,
                                      **kwargs) -> np.ndarray:
    if len(trackers) == 0 or len(detections) == 0:
        return []

    cost_mat, iou_mat = cost_matrix_iou_feature_for_panorama(trackers, detections, image_width, **kwargs)
    row_ind, col_ind = scipy.optimize.linear_sum_assignment(cost_mat)

    matches = []
    for r, c in zip(row_ind, col_ind):
        # check linear assignment winner
        if iou_mat[r, c] >= min_iou:
            matches.append((r, c))

        # check other high IOU detections
        if multi_match_min_iou < 1.:
            for c2 in range(iou_mat.shape[1]):
                if c2 != c and iou_mat[r, c2] > multi_match_min_iou:
                    matches.append((r, c2))

    return np.array(matches)


class PanoramaIOUAndFeatureMatchingFunction(BaseMatchingFunction):
    """ class implements the basic matching function, taking into account
    detection boxes overlap measured using IOU metric and optional 
    feature similarity measured with a specified metric """

    def __init__(self,
                 image_width,
                 min_iou: float = 0.1,
                 multi_match_min_iou: float = 1. + EPS,
                 feature_similarity_fn: Callable = angular_similarity,
                 feature_similarity_beta: Optional[float] = None) -> None:
        self.image_width = image_width
        self.min_iou = min_iou
        self.multi_match_min_iou = multi_match_min_iou
        self.feature_similarity_fn = feature_similarity_fn
        self.feature_similarity_beta = feature_similarity_beta

    def __call__(self,
                 trackers: Sequence[SingleObjectTracker],
                 detections: Sequence[Detection]) -> np.ndarray:
        return match_by_cost_matrix_for_panorama(
            trackers, detections, self.image_width,
            min_iou=self.min_iou,
            multi_match_min_iou=self.multi_match_min_iou,
            feature_similarity_fn=self.feature_similarity_fn,
            feature_similarity_beta=self.feature_similarity_beta)


class PanoramaMultiObjectTracker:
    def __init__(self, dt: float,
                 image_width: int,
                 model_spec: Union[str, Dict] = DEFAULT_MODEL_SPEC,
                 matching_fn: Optional[BaseMatchingFunction] = None,
                 tracker_kwargs: Dict = None,
                 matching_fn_kwargs: Dict = None,
                 active_tracks_kwargs: Dict = None) -> None:
        self.trackers: List[SingleObjectTracker] = []

        # kwargs to be passed to each single object tracker
        self.tracker_kwargs: Dict = tracker_kwargs if tracker_kwargs is not None else {}
        self.tracker_clss: Optional[Type[SingleObjectTracker]] = None

        # translate model specification into single object tracker to be used
        if model_spec is None:
            self.tracker_clss = SimpleTracker
            if dt is not None:
                logger.warning(
                    'specified dt is ignored in simple tracker mode')
        elif isinstance(model_spec, dict):
            self.tracker_clss = KalmanTracker
            self.tracker_kwargs['model_kwargs'] = model_spec
            self.tracker_kwargs['model_kwargs']['dt'] = dt
        elif isinstance(model_spec, str) and model_spec in ModelPreset.__members__:
            self.tracker_clss = KalmanTracker
            self.tracker_kwargs['model_kwargs'] = ModelPreset[model_spec].value
            self.tracker_kwargs['model_kwargs']['dt'] = dt
        else:
            raise NotImplementedError(f'unsupported motion model {model_spec}')

        logger.debug(
            f'using single tracker of class: {self.tracker_clss} with kwargs: {self.tracker_kwargs}')

        self.matching_fn: BaseMatchingFunction = matching_fn
        self.matching_fn_kwargs: Dict = matching_fn_kwargs if matching_fn_kwargs is not None else {}
        self.matching_fn_kwargs['image_width'] = image_width
        if self.matching_fn is None:
            self.matching_fn = PanoramaIOUAndFeatureMatchingFunction(
                **self.matching_fn_kwargs)

        # kwargs to be used when self.step returns active tracks
        self.active_tracks_kwargs: Dict = active_tracks_kwargs if active_tracks_kwargs is not None else {}
        logger.debug('using active_tracks_kwargs: %s' %
                     str(self.active_tracks_kwargs))

    def active_tracks(self,
                      max_staleness_to_positive_ratio: float = 3.0,
                      max_staleness: float = 999,
                      min_steps_alive: int = -1) -> List[Track]:
        """ returns all active tracks after optional filtering by tracker steps count and staleness """

        tracks: List[Track] = []
        for tracker in self.trackers:
            cond1 = tracker.staleness / \
                tracker.steps_positive < max_staleness_to_positive_ratio  # early stage
            cond2 = tracker.staleness < max_staleness
            cond3 = tracker.steps_alive >= min_steps_alive
            if cond1 and cond2 and cond3:
                tracks.append(Track(id=tracker.id, box=tracker.box(
                ), score=tracker.score, class_id=tracker.class_id))

        logger.debug('active/all tracks: %d/%d' %
                     (len(self.trackers), len(tracks)))
        return tracks

    def cleanup_trackers(self) -> None:
        count_before = len(self.trackers)
        self.trackers = [t for t in self.trackers if not (
            t.is_stale() or t.is_invalid())]
        count_after = len(self.trackers)
        logger.debug('deleted %s/%s trackers' %
                     (count_before - count_after, count_before))

    def step(self, detections: Sequence[Detection]) -> List[Track]:
        """ the method matches the new detections with existing trackers,
        creates new trackers if necessary and performs the cleanup.
        Returns the active tracks after active filtering applied """
        t0 = time.time()

        # filter out empty detections
        detections = [det for det in detections if det.box is not None]

        # predict state in all trackers
        for t in self.trackers:
            t.predict()

        # match trackers with detections
        logger.debug('step with %d detections' % len(detections))
        matches = self.matching_fn(self.trackers, detections)
        logger.debug('matched %d pairs' % len(matches))

        # assigned trackers: correct
        for match in matches:
            track_idx, det_idx = match[0], match[1]
            self.trackers[track_idx].update(detection=detections[det_idx])

        # not assigned detections: create new trackers POF
        assigned_det_idxs = set(matches[:, 1]) if len(matches) > 0 else []
        for det_idx in set(range(len(detections))).difference(assigned_det_idxs):
            det = detections[det_idx]
            tracker = self.tracker_clss(box0=det.box,
                                        score0=det.score,
                                        class_id0=det.class_id,
                                        **self.tracker_kwargs)
            self.trackers.append(tracker)

        # unassigned trackers
        assigned_track_idxs = set(matches[:, 0]) if len(matches) > 0 else []
        for track_idx in set(range(len(self.trackers))).difference(assigned_track_idxs):
            self.trackers[track_idx].stale()

        # cleanup dead trackers
        self.cleanup_trackers()

        # log step timing
        elapsed = (time.time() - t0) * 1000.
        logger.debug(f'tracking step time: {elapsed:.3f} ms')

        return self.active_tracks(**self.active_tracks_kwargs)
