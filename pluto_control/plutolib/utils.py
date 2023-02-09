import numpy as np
from plutolib.kalman import KalmanFilter


class Filter:
    def __init__(self, r) -> None:
        self.dt = 1.0 / 60
        self.r = 0.5 if r == None else r
        self.F, self.H, self.Q, self.R, self.kf = self.make_kalman(
            self.dt
        )  # For X of marker 1

    def make_kalman(self, dtime):
        F = np.array([[1, dtime, 0], [0, 1, dtime], [0, 0, 1]])
        H = np.array([1, 0, 0]).reshape(1, 3)
        Q = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
        R = np.array([self.r]).reshape(1, 1)
        kf = KalmanFilter(F=F, H=H, Q=Q, R=R)
        return F, H, Q, R, kf

    # ---------------------------------------------------------
    # | Function to update according to predictions by Kalman |
    # ---------------------------------------------------------
    def predict_kalman(self, coordinate):
        coordinate_unfiltered = coordinate
        coordinate = np.dot(self.H, self.kf.predict())[0]
        self.kf.update(coordinate_unfiltered)
        return coordinate
