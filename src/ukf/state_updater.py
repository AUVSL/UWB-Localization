import numpy as np

from ukf.datapoint import DataType
from ukf.state import UKFState
from ukf.util import normalize


class StateUpdater:
    def __init__(self, NX, N_SIGMA, WEIGHTS):
        self.N_SIGMA = N_SIGMA
        self.WEIGHTS = WEIGHTS
        self.NX = NX

    def compute_Tc(self, predicted_x, predicted_z, sigma_x, sigma_z):
        dx = np.subtract(sigma_x.T, predicted_x).T

        normalize(dx, UKFState.YAW)

        dz = np.subtract(sigma_z.T, predicted_z)

        normalize(dz, UKFState.YAW)

        return np.matmul(self.WEIGHTS * dx, dz)

    def update(self, z, S, Tc, predicted_z, predicted_x, predicted_P, data_type):
        Si = np.linalg.inv(S)
        K = np.matmul(Tc, Si)

        dz = z - predicted_z

        if (data_type == DataType.ODOMETRY):
            normalize(dz, UKFState.YAW)

            # print(z[UKFState.YAW], predicted_z[UKFState.YAW], dz[UKFState.YAW])

        # Dm = np.sqrt(np.matmul(np.matmul(dz, Si), dz))

        self.x = predicted_x + np.matmul(K, dz)
        self.P = predicted_P - np.matmul(K, np.matmul(S, K.transpose()))
        self.nis = np.matmul(dz.transpose(), np.matmul(Si, dz))

    def process(self, predicted_x, predicted_z, z, S, predicted_P, sigma_x, sigma_z, data_type):
        Tc = self.compute_Tc(predicted_x, predicted_z, sigma_x, sigma_z)
        self.update(z, S, Tc, predicted_z, predicted_x, predicted_P, data_type)

        normalize(self.x, UKFState.YAW)
