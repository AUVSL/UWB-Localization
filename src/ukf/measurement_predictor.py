import numpy as np

from ukf.datapoint import DataType
from ukf.state import UKFState
from ukf.util import normalize


class MeasurementPredictor:
    def __init__(self, sensor_std, N_SIGMA, WEIGHTS):
        self.sensor_std = sensor_std

        self.compute_R_matrix()

        self.WEIGHTS = WEIGHTS
        self.N_SIGMA = N_SIGMA

        self.z = None
        self.S = None
        self.sigma_z = None
        self.current_type = None

        self.R = None
        self.nz = None

    def initialize(self, data):
        sensor_type = data.data_type

        self.current_type = sensor_type

        self.R = self.sensor_std[sensor_type]["R"]
        self.nz = self.sensor_std[sensor_type]['nz']

        if sensor_type == DataType.UWB:
            self.anchor_pos = data.extra['anchor']
            self.sensor_offset = data.extra['sensor_offset']

    def rotation_matrix(self, angle):
        s = np.sin(angle)
        c = np.cos(angle)

        return [[c, -s, 0],
                [s, c, 0],
                [0, 0, 1]]

    def compute_sigma_z(self, sigma_x):
        sigma = np.zeros((self.nz, self.N_SIGMA))

        if self.current_type == DataType.LIDAR:
            sigma[UKFState.X] = sigma_x[UKFState.X]  # px
            sigma[UKFState.Y] = sigma_x[UKFState.Y]  # py
        elif self.current_type == DataType.UWB:
            sensor_pose = np.copy(sigma_x[:UKFState.Z + 1])

            if self.sensor_offset is not None:
                angles = np.unique(sigma_x[UKFState.YAW])

                for angle in angles:
                    rot = self.rotation_matrix(angle)
                    offset_rot = np.matmul(rot, self.sensor_offset).reshape((-1, 1))

                    sensor_pose[:, sigma_x[UKFState.YAW] == angle] += offset_rot

            distances = np.linalg.norm(sensor_pose - self.anchor_pos.reshape((-1, 1)), axis=0)
            sigma[0] = distances
        elif self.current_type == DataType.ODOMETRY:
            sigma[UKFState.X] = sigma_x[UKFState.X]  # px
            sigma[UKFState.Y] = sigma_x[UKFState.Y]  # py
            sigma[UKFState.Z] = sigma_x[UKFState.Z]  # pz
            sigma[UKFState.V] = sigma_x[UKFState.V]  # v
            sigma[UKFState.YAW] = sigma_x[UKFState.YAW]  # theta
            sigma[UKFState.YAW_RATE] = sigma_x[UKFState.YAW_RATE]  # theta_yaw

        return sigma

    def compute_z(self, sigma):
        return np.dot(sigma, self.WEIGHTS)

    def compute_S(self, sigma, z):
        sub = np.subtract(sigma.T, z).T

        if self.current_type == DataType.ODOMETRY:
            normalize(sub, UKFState.YAW)

        return (np.matmul(self.WEIGHTS * sub, sub.T)) + self.R

    def process(self, sigma_x, data):
        self.initialize(data)
        self.sigma_z = self.compute_sigma_z(sigma_x)
        self.z = self.compute_z(self.sigma_z)
        self.S = self.compute_S(self.sigma_z, self.z)

    def compute_R_matrix(self):
        for value in self.sensor_std:
            self.sensor_std[value]["R"] = np.diag(np.power(self.sensor_std[value]['std'], 2))
