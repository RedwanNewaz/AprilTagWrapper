import numpy as np
import math
import pickle as pkl

FPS = 23.0
DT = 1.0/FPS
# Covariance for EKF simulation
Q = np.diag([
    0.20,  # variance of location on x-axis
    0.20,  # variance of location on y-axis
    np.deg2rad(1.0),  # variance of yaw angle
    0.050  # variance of velocity
]) ** 2  # predict state covariance
R = np.diag([2.10, 2.10]) ** 2  # Observation x,y position covariance

def motion_model(x, u):
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F @ x + B @ u

    return x

def observation_model(x):
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = H @ x

    return z

def ekf_estimation(xEst, PEst, z, u):
    def jacob_f(x, u):
        """
        Jacobian of Motion Model
        motion model
        x_{t+1} = x_t+v*dt*cos(yaw)
        y_{t+1} = y_t+v*dt*sin(yaw)
        yaw_{t+1} = yaw_t+omega*dt
        v_{t+1} = v{t}
        so
        dx/dyaw = -v*dt*sin(yaw)
        dx/dv = dt*cos(yaw)
        dy/dyaw = v*dt*cos(yaw)
        dy/dv = dt*sin(yaw)
        """
        yaw = x[2, 0]
        v = u[0, 0]
        jF = np.array([
            [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
            [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw)],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])

        return jF
    def jacob_h():
        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        return jH
    #  Predict
    xPred = motion_model(xEst, u)
    jF = jacob_f(xEst, u)
    PPred = jF @ PEst @ jF.T + Q

    #  Update
    jH = jacob_h()
    zPred = observation_model(xPred)
    y = z - zPred
    S = jH @ PPred @ jH.T + R
    K = PPred @ jH.T @ np.linalg.inv(S)
    xEst = xPred + K @ y
    PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
    return xEst, PEst

class EKF(object):
    def __init__(self, frame_rate):
        global DT
        DT = 1.0 / float(frame_rate)
        self.xEst = np.zeros((4, 1))
        self.PEst = np.eye(4)
        self.u = np.array([[0], [0]])
        self.initialize = False

    def calc_input(self, old, new):
        delta = new - old
        delta = delta[:2]
        delta[0] = new[-1] # because we are tracking linera velocity -> [performance improved]
        return delta


    def __call__(self, *args, **kwargs):
        if not self.initialize:
            return

        self.xEst, self.PEst = ekf_estimation(self.xEst, self.PEst, self.z, self.u)
        self.u = self.calc_input(self.xEst_prev, self.xEst)
        mean = np.squeeze(self.xEst)
        return mean

    def update(self, x, y):
        self.z = np.array([[x], [y]])
        if not self.initialize:
            self.xEst[0, 0] = self.z[0, 0]
            self.xEst[1, 0] = self.z[1, 0]
            self.initialize = True

        self.xEst_prev = self.xEst.copy()
        self.xEst, self.PEst = ekf_estimation(self.xEst, self.PEst, self.z, self.u)




if __name__ == '__main__':
    data = pkl.load(open("../Test/trajectory.pkl", "rb"))
    data = np.asarray(data)

    import matplotlib.pyplot as plt
    plt.figure(figsize=(16, 10))
    plt.axis([0, 1280, 0, 720])

    #simulation parameters
    clock = data[0][0]
    sim_time = data[-1][0]
    N = len(data)
    index = 0

    #create EKF
    ekf = EKF()
    ekf.update(data[index][1], data[index][2])

    # run simulations
    while clock < sim_time:
        clock += DT
        if index + 1 < N and clock > data[index][0]:
            index += 1
            ekf.update(data[index][1], data[index][2])
            plt.scatter(data[index][1], data[index][2], color='k', s=5)

        mean = ekf()
        plt.scatter(mean[0], mean[1], color='r', s=5)
        plt.pause(0.01)



