import numpy as np


class TrackingExtension(object):

    def __init__(self):
        self._prev_dets_xy = None
        self._prev_dets_cls = None
        self._prev_instance_mask = None
        self._prev_dets_to_tracks = None
        # a list of track id for each detection

        self._tracks = []
        self._tracks_cls = []
        self._tracks_age = []

        self._max_track_age = 100
        self._max_assoc_dist = 0.7

    def __call__(self, dets_xy, dets_cls, instance_mask, sim_matrix):
        # first frame
        if self._prev_dets_xy is None:
            self._prev_dets_xy = dets_xy
            self._prev_dets_cls = dets_cls
            self._prev_instance_mask = instance_mask
            self._prev_dets_to_tracks = np.arange(len(dets_xy), dtype=np.int32)

            for d_xy, d_cls in zip(dets_xy, dets_cls):
                self._tracks.append([d_xy])
                self._tracks_cls.append([np.asscalar(d_cls)])
                self._tracks_age.append(0)

            return

        # associate detections
        prev_dets_inds = self._associate_prev_det(
            dets_xy, dets_cls, instance_mask, sim_matrix)

        # mapping from detection indices to tracklets indices
        dets_to_tracks = []

        # assign current detections to tracks based on assocation with previous
        # detections
        for d_idx, (d_xy, d_cls, prev_d_idx) in enumerate(
                zip(dets_xy, dets_cls, prev_dets_inds)):
            # distance between assocated detections
            dxy = self._prev_dets_xy[prev_d_idx] - d_xy
            dxy = np.hypot(dxy[0], dxy[1])

            if dxy < self._max_assoc_dist and prev_d_idx >= 0:
                # if current detection is close to the associated detection,
                # append to the tracklet
                ti = self._prev_dets_to_tracks[prev_d_idx]
                self._tracks[ti].append(d_xy)
                self._tracks_cls[ti].append(np.asscalar(d_cls))
                self._tracks_age[ti] = -1
                dets_to_tracks.append(ti)
            else:
                # otherwise start a new tracklet
                self._tracks.append([d_xy])
                self._tracks_cls.append([np.asscalar(d_cls)])
                self._tracks_age.append(-1)
                dets_to_tracks.append(len(self._tracks) - 1)

        # tracklet age
        # pop_inds = []
        for i in range(len(self._tracks_age)):
            self._tracks_age[i] += 1
        #     if self._tracks_age[i] > self._max_track_age:
        #         pop_inds.append(i)

        # if len(pop_inds) > 0:
        #     pop_inds.reverse()
        #     for pi in pop_inds:
        #         for j in range(len(dets_to_tracks)):
        #             if dets_to_tracks[j] == pi:
        #                 dets_to_tracks[j] = -1
        #             elif dets_to_tracks[j] > pi:
        #                 dets_to_tracks[j] = dets_to_tracks[j] - 1
        #         self._tracks.pop(pi)
        #         self._tracks_cls.pop(pi)
        #         self._tracks_age.pop(pi)

        # update
        self._prev_dets_xy = dets_xy
        self._prev_dets_cls = dets_cls
        self._prev_instance_mask = instance_mask
        self._prev_dets_to_tracks = dets_to_tracks

    def get_tracklets(self, only_latest_track=False):
        if only_latest_track is True:
            ids = self._prev_dets_to_tracks
        else:
            ids = range(len(self._tracks))
        tracks, tracks_cls, track_ids = [], [], []
        for i in ids:
            if self._tracks_age[i] < self._max_track_age:
                tracks.append(np.stack(self._tracks[i], axis=0))
                tracks_cls.append(np.array(self._tracks_cls[i]).mean())
                track_ids.append(i)
        return tracks, tracks_cls, track_ids

    def _associate_prev_det(self, dets_xy, dets_cls, instance_mask, sim_matrix):
        prev_dets_inds = []
        occupied_flag = np.zeros(len(self._prev_dets_xy), dtype=np.bool)
        sim = sim_matrix[0]
        for d_idx, (d_xy, d_cls) in enumerate(zip(dets_xy, dets_cls)):
            inst_id = d_idx + 1  # instance is 1-based

            # For all the points that belong to the current instance,
            # find their most similar points in the previous scans and take
            # the point with highest support as the associated point of this
            # instance in the previous scan.
            inst_sim = sim[instance_mask == inst_id].argmax(axis=1)
            assoc_prev_pt_inds = np.bincount(inst_sim).argmax()

            # associated detection
            prev_d_idx = self._prev_instance_mask[assoc_prev_pt_inds] - 1
            # instance is 1-based

            # only associate one detection
            if occupied_flag[prev_d_idx]:
                prev_dets_inds.append(-1)
            else:
                prev_dets_inds.append(prev_d_idx)
                occupied_flag[prev_d_idx] = True

        return prev_dets_inds
