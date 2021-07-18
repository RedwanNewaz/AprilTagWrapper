import numpy as np

class FilterBase:
    def __init__(self, F, B, H):
        self.F = F
        self.B = B
        self.H = H
    def motion_model(self, x, u):
        x = self.F @ x + self.B @ u
        return x
    def observation_model(self, x):
        z = self.H @ x
        return z

class ExtendedKalmanFilterBase(FilterBase):
    def __init__(self, F, B, H):
        super().__init__(F, B, H)
        self.initialization = False

    def set(self, xEst, PEst, Q, R):
        '''
        initial position and covariance
        :param xEst: state vector
        :param PEst: covariance matrix
        '''
        self.xEst = xEst
        self.PEst = PEst
        self.Q = Q
        self.R = R
        self.initialization = True

    def jacob_h(self):
        raise NotImplemented

    def jacob_f(self, x, u):
        raise NotImplemented

    def update(self, z, u):

        assert self.initialization, "EKF is not initialized !"

        xEst, PEst = self.xEst, self.PEst
        Q, R = self.Q, self.R
        #  Predict
        xPred = self.motion_model(xEst, u)
        jF = self.jacob_f(xEst, u)
        PPred = jF @ PEst @ jF.T + Q

        #  Update
        jH = self.jacob_h()
        zPred = self.observation_model(xPred)
        y = z - zPred
        S = jH @ PPred @ jH.T + R
        K = PPred @ jH.T @ np.linalg.inv(S)
        self.xEst = xPred + K @ y
        self.PEst = (np.eye(len(xEst)) - K @ jH) @ PPred



