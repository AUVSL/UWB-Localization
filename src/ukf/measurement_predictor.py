# coding=utf-8
import numpy as np

from ukf.datapoint import DataType
from ukf.state import UKFState
from ukf.util import angle_diff


class MeasurementPredictor(object):
    def __init__(self, sensor_std, N_SIGMA, WEIGHTS_M, WEIGHTS_C):
        self.WEIGHTS_C = WEIGHTS_C
        self.sensor_std = sensor_std

        self.compute_R_matrix()

        self.WEIGHTS_M = WEIGHTS_M
        self.N_SIGMA = N_SIGMA

        self.z = None
        self.S = None
        self.sigma_z = None
        self.current_type = None

        self.R = None
        self.nz = None
        self.anchor_pos = None
        self.sensor_offset = None

    def initialize(self, data):
        sensor_type = data.data_type

        self.current_type = sensor_type

        self.R = self.sensor_std[sensor_type]["R"]
        self.nz = self.sensor_std[sensor_type]['nz']

        if sensor_type == DataType.UWB:
            self.anchor_pos = data.extra['anchor']
            self.sensor_offset = data.extra['sensor_offset']

    def rotation_matrix(self, angle):
        output = np.zeros((3, 3, angle.size))

        s = np.sin(angle)
        c = np.cos(angle)

        output[0, 0, :] = c
        output[1, 0, :] = s
        output[0, 1, :] = -s
        output[1, 1, :] = c
        output[2, 2, :] = 1

        return output

    def compute_sigma_z(self, sigma_x):
        sigma = np.zeros((self.nz, self.N_SIGMA))

        if self.current_type == DataType.LIDAR:
            sigma[UKFState.X] = sigma_x[UKFState.X]  # px
            sigma[UKFState.Y] = sigma_x[UKFState.Y]  # py
        elif self.current_type == DataType.UWB:
            sensor_pose = sigma_x[:UKFState.Z + 1]

            if self.sensor_offset is not None:
                angles = sigma_x[UKFState.YAW]
                rot = self.rotation_matrix(angles)

                offsets = np.einsum('ijn,j->in', rot, self.sensor_offset)

                sensor_pose = sensor_pose + offsets

            distances = np.linalg.norm(sensor_pose - self.anchor_pos.reshape((-1, 1)), axis=0)
            sigma[0] = distances
        elif self.current_type == DataType.ODOMETRY:
            sigma[UKFState.X] = sigma_x[UKFState.X]  # px
            sigma[UKFState.Y] = sigma_x[UKFState.Y]  # py
            sigma[UKFState.Z] = sigma_x[UKFState.Z]  # pz
            sigma[UKFState.V] = sigma_x[UKFState.V]  # v
            sigma[UKFState.YAW] = sigma_x[UKFState.YAW]  # theta
            sigma[UKFState.YAW_RATE] = sigma_x[UKFState.YAW_RATE]  # theta_yaw
        elif self.current_type == DataType.IMU:
            sigma[0] = sigma_x[UKFState.YAW]  # theta

        return sigma

    def compute_z(self, sigma):
        return np.dot(sigma, self.WEIGHTS_M)

    def compute_S(self, sigma, z):
        sub = np.subtract(sigma.T, z).T

        if self.current_type == DataType.ODOMETRY:
            sub[UKFState.YAW] = angle_diff(sigma[UKFState.YAW], z[UKFState.YAW])
        elif self.current_type == DataType.IMU:
            sub = angle_diff(sigma, z)

        return (np.matmul(self.WEIGHTS_C * sub, sub.T)) + self.R

    def process(self, sigma_x, data):
        self.initialize(data)
        self.sigma_z = self.compute_sigma_z(sigma_x)
        self.z = self.compute_z(self.sigma_z)
        self.S = self.compute_S(self.sigma_z, self.z)

    def compute_R_matrix(self):
        for value in self.sensor_std:
            self.sensor_std[value]["R"] = np.diag(np.power(self.sensor_std[value]['std'], 2))
