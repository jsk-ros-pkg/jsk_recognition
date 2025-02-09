from __future__ import division
from __future__ import print_function

import numpy as np
import rospy

from jsk_recognition_utils.matching import min_cost_matching


class Sort(object):

    def __init__(self, tracker_class, distance_metric,
                 max_distance, max_age=1):
        self.tracker_class = tracker_class
        self.distance_metric = distance_metric
        self.max_distance = max_distance
        self.max_age = max_age
        self.trackers = []
        self.track_id = 0

    def update(self, detections):
        # get predicted locations from existing trackers.
        tracks = []
        to_del = []
        for t, tracker in enumerate(self.trackers):
            pos = tracker.predict()
            tracks.append(np.array(pos).reshape(-1))
            if np.any(np.isnan(pos)):
                to_del.append(t)
        if len(tracks) > 0:
            tracks = np.ma.compress_rows(np.ma.masked_invalid(tracks))
        for t in reversed(to_del):
            self.trackers.pop(t)
        matched, unmatched_tracks, unmatched_dets = min_cost_matching(
            self.distance_metric, self.max_distance, tracks, detections)

        rospy.logdebug("Sort's min_cost_matching results.")
        rospy.logdebug('len(matched) = {}'.format(len(matched)))
        rospy.logdebug('len(unmatched_dets) = {}'.format(len(unmatched_dets)))
        rospy.logdebug('len(unmatched_tracks) = {}'.format(
            len(unmatched_tracks)))

        # update matched trackers with assigned detections
        for matched_detection_id, matched_track_id in matched:
            self.trackers[matched_track_id].update(
                detections[matched_detection_id, :])

        # create and initialise new trackers for unmatched detections
        for i in unmatched_dets:
            trk = self.tracker_class(detections[i, :], track_id=self.track_id)
            self.track_id += 1
            self.trackers.append(trk)
        i = len(self.trackers)
        for trk in reversed(self.trackers):
            i -= 1
            # remove dead tracklet
            if trk.time_since_update > self.max_age:
                self.trackers.pop(i)

    def get_tracklets(self):
        tracks, track_ids = [], []
        for tracker in self.trackers:
            if tracker.time_since_update > 1:
                continue
            tracks.append(
                np.stack(tracker.history, axis=0))
            track_ids.append(tracker.track_id)
        return tracks, track_ids
