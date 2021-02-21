import numpy as np
from datapoint import DataType


class StateUpdater:
    def __init__(self, NX, N_SIGMA, WEIGHTS):
        self.N_SIGMA = N_SIGMA
        self.WEIGHTS = WEIGHTS
        self.NX = NX

    def compute_Tc(self, predicted_x, predicted_z, sigma_x, sigma_z):
        dx = np.subtract(sigma_x.T, predicted_x).T

        dx[4] %= (2 * np.pi)
        mask = np.abs(dx[4]) > np.pi
        dx[4, mask] -= (np.pi * 2)

        dz = np.subtract(sigma_z.T, predicted_z)

        dz[4] %= 2 * np.pi
        mask = np.abs(dz[4]) > np.pi
        dz[4, mask] -= (np.pi * 2)

        return np.matmul(self.WEIGHTS * dx, dz)

    def update(self, z, S, Tc, predicted_z, predicted_x, predicted_P, data_type):
        Si = np.linalg.inv(S)
        K = np.matmul(Tc, Si)

        dz = z - predicted_z

        if(data_type == DataType.ODOMETRY):
            dz[4] = (dz[4] + np.pi) % (2 * np.pi) - np.pi 
            
        # Dm = np.sqrt(np.matmul(np.matmul(dz, Si), dz))

        self.x = predicted_x + np.matmul(K, dz)
        self.P = predicted_P - np.matmul(K, np.matmul(S, K.transpose()))
        self.nis = np.matmul(dz.transpose(), np.matmul(Si, dz))

    def process(self, predicted_x, predicted_z, z, S, predicted_P, sigma_x, sigma_z, data_type):
        Tc = self.compute_Tc(predicted_x, predicted_z, sigma_x, sigma_z)
        self.update(z, S, Tc, predicted_z, predicted_x, predicted_P, data_type)

        self.x[4] %= 2 * np.pi
        if self.x[4] > np.pi:
            self.x[4] -= (2 * np.pi)

        self.P[4] %= 2 * np.pi
        mask = np.abs(self.P[4]) > np.pi
        self.P[4, mask] -= (np.pi * 2)
