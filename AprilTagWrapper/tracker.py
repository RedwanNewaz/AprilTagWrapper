import numpy as np
import math
from .FilterPool import ExtendedKalmanFilterBase


class EKFImageCoord(ExtendedKalmanFilterBase):
    EIC_F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    EIC_H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    EIC_Q = np.diag([
        0.50,  # variance of location on x-axis
        0.50,  # variance of location on y-axis
        np.deg2rad(1.0),  # variance of yaw angle
        0.50  # variance of velocity
    ]) ** 2  # predict state covariance
    EIC_R = np.diag([2.10, 2.10]) ** 2  # Observation x,y position covariance

    def __init__(self, FPS):
        self.dt = 1.0/float(FPS)
        EIC_B = np.array([[1.0, 0],
                          [1.0, 0],
                          [0.0, self.dt],
                          [1.0, 0.0]])
        super().__init__(self.EIC_F, EIC_B, self.EIC_H)
    def jacob_h(self):
        return self.EIC_H

    def jacob_f(self, x, u):
        yaw = x[2, 0]
        v = u[0, 0]
        jF = np.array([
            [1.0, 0.0, -self.dt * v * math.sin(yaw), self.dt * math.cos(yaw)],
            [0.0, 1.0, self.dt * v * math.cos(yaw), self.dt * math.sin(yaw)],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])

        return jF

    def __call__(self, x, y):
        self.z = np.array([[x], [y]])
        if not self.initialization:
            self.u = np.array([[0], [0]])
            xEst = np.zeros((4, 1))
            PEst = np.eye(4)
            xEst[0, 0] = x
            xEst[1, 0] = y
            self.set(xEst, PEst, self.EIC_Q, self.EIC_R)
            return

        yaw = self.xEst[2, 0]

        self.u = (self.xEst[:2] * self.dt - self.u)
        self.B[0, 0] = math.cos(yaw) * self.dt
        self.B[1, 0] = math.sin(yaw) * self.dt

        self.update(self.z, self.u)

    def __next__(self):
        if not self.initialization:
            return
        self.update(self.z, self.u)
        mean = np.squeeze(self.xEst)
        return mean




