from filterpy.kalman import KalmanFilter
import numpy as np


class KalmanPositionTracker(object):

    def __init__(self, xyz, track_id=0):
        # define constant velocity model
        # X,Y,Z, dX, dY, dZ
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.kf.F = np.array(
            [[1, 0, 0, 1, 0, 0],
             [0, 1, 0, 0, 1, 0],
             [0, 0, 1, 0, 0, 1],
             [0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]])
        self.kf.H = np.array(
            [[1, 0, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0],
             [0, 0, 1, 0, 0, 0]])

        self.kf.R[3:, 3:] *= 10.
        # give high uncertainty to the unobservable initial velocities
        self.kf.P[3:, 3:] *= 1000.0
        self.kf.P *= 10.
        self.kf.Q[3:, 3:] *= 0.01
        self.kf.R *= 0.01

        xyz = np.array(xyz, 'f')
        self.kf.x[:3] = xyz.reshape(-1, 1)
        self.time_since_update = 0
        self.track_id = track_id
        self.history = [xyz.reshape(-1, 1)]

    def update(self, xyz):
        xyz = np.array(xyz, 'f')
        self.time_since_update = 0
        self.history.append(xyz.reshape(-1, 1))
        self.kf.update(xyz.squeeze())

    def predict(self):
        self.kf.predict()
        self.time_since_update += 1
        return self.kf.x[:3]

    def get_state(self):
        return self.kf.x

    def get_history(self):
        return self.history
