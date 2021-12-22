from motpy.core import Box, Detection, Track, Vector, setup_logger
from motpy.metrics import angular_similarity, calculate_iou
from motpy.model import Model, ModelPreset

logger = setup_logger(__name__)

class MultiObjectTracker:
    def __init__(self, dt: float, image_width: int,
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
                logger.warning('specified dt is ignored in simple tracker mode')
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

        logger.debug(f'using single tracker of class: {self.tracker_clss} with kwargs: {self.tracker_kwargs}')

        self.matching_fn: BaseMatchingFunction = matching_fn
        self.matching_fn_kwargs: Dict = matching_fn_kwargs if matching_fn_kwargs is not None else {}
        if self.matching_fn is None:
            self.matching_fn = IOUAndFeatureMatchingFunction(**self.matching_fn_kwargs)

        # kwargs to be used when self.step returns active tracks
        self.active_tracks_kwargs: Dict = active_tracks_kwargs if active_tracks_kwargs is not None else {}
        logger.debug('using active_tracks_kwargs: %s' % str(self.active_tracks_kwargs))

    def active_tracks(self,
                      max_staleness_to_positive_ratio: float = 3.0,
                      max_staleness: float = 999,
                      min_steps_alive: int = -1) -> List[Track]:
        """ returns all active tracks after optional filtering by tracker steps count and staleness """

        tracks: List[Track] = []
        for tracker in self.trackers:
            cond1 = tracker.staleness / tracker.steps_positive < max_staleness_to_positive_ratio  # early stage
            cond2 = tracker.staleness < max_staleness
            cond3 = tracker.steps_alive >= min_steps_alive
            if cond1 and cond2 and cond3:
                tracks.append(Track(id=tracker.id, box=tracker.box(), score=tracker.score, class_id=tracker.class_id))

        logger.debug('active/all tracks: %d/%d' % (len(self.trackers), len(tracks)))
        return tracks

    def cleanup_trackers(self) -> None:
        count_before = len(self.trackers)
        self.trackers = [t for t in self.trackers if not (t.is_stale() or t.is_invalid())]
        count_after = len(self.trackers)
        logger.debug('deleted %s/%s trackers' % (count_before - count_after, count_before))

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
