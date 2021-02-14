import numpy as np


class StateUpdater:
    def __init__(self, NX, N_SIGMA, WEIGHTS) -> None:
        super().__init__()
        self.N_SIGMA = N_SIGMA
        self.WEIGHTS = WEIGHTS
        self.NX = NX

    def compute_Tc(self, predicted_x, predicted_z, sigma_x, sigma_z):
        dx = np.subtract(sigma_x.T, predicted_x).T
        mask = np.abs(dx[3]) > np.pi
        dx[3, mask] = dx[3, mask] % (np.pi * 2)

        dz = np.subtract(sigma_z.T, predicted_z)

        return np.matmul(self.WEIGHTS * dx, dz)

    def update(self, z, S, Tc, predicted_z, predicted_x, predicted_P):
        Si = np.linalg.inv(S)
        K = np.matmul(Tc, Si)

        dz = z - predicted_z

        self.x = predicted_x + np.matmul(K, dz)
        self.P = predicted_P - np.matmul(K, np.matmul(S, K.transpose()))
        self.nis = np.matmul(dz.transpose(), np.matmul(Si, dz))

    def process(self, predicted_x, predicted_z, z, S, predicted_P, sigma_x, sigma_z):
        Tc = self.compute_Tc(predicted_x, predicted_z, sigma_x, sigma_z)
        self.update(z, S, Tc, predicted_z, predicted_x, predicted_P)
