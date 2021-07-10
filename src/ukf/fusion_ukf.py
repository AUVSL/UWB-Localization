# coding=utf-8
import numpy as np

from ukf.measurement_predictor import MeasurementPredictor
from ukf.state_predictor import StatePredictor
from ukf.state_updater import StateUpdater


class FusionUKF(object):
    def __init__(self, sensor_std, speed_noise_std=.9, yaw_rate_noise_std=.6, alpha=1, beta=0, k=None):
        # ODOMETRY: beta=0.3
        # UWB: beta=0.3

        self.initialized = False
        self.timestamp = None

        # Number of total states X, Y, Z, velocity, yaw, yaw rate
        self.NX = 6

        # Settings values -----------------------------------
        self.N_AUGMENTED = self.NX + 2

        if k is None:
            self.K = 3 - self.N_AUGMENTED
        else:
            self.K = k

        self.N_SIGMA = self.N_AUGMENTED * 2 + 1
        self.ALPHA = alpha
        self.LAMBDA = (self.ALPHA ** 2) * (self.NX + self.K) - self.NX
        self.SCALE = np.sqrt(self.LAMBDA + self.N_AUGMENTED)
        self.W = 0.5 / (self.LAMBDA + self.N_AUGMENTED)
        self.W0_M = self.LAMBDA / (self.LAMBDA + self.N_AUGMENTED)
        self.W0_C = self.LAMBDA / (self.LAMBDA + self.N_AUGMENTED) + (1 - alpha ** 2 + beta)

        self.WEIGHTS_M = np.full(self.N_SIGMA, self.W)
        self.WEIGHTS_M[0] = self.W0_M

        self.WEIGHTS_C = np.full(self.N_SIGMA, self.W)
        self.WEIGHTS_C[0] = self.W0_C
        # -----------------------------------

        # Uncertainty Settings -----------------------------------
        self.SPEED_NOISE_STD = speed_noise_std
        self.YAW_RATE_NOISE_STD = yaw_rate_noise_std

        self.SPEED_NOISE_VAR = self.SPEED_NOISE_STD ** 2
        self.YAW_RATE_NOISE_VAR = self.YAW_RATE_NOISE_STD ** 2
        # -----------------------------------

        # Measurement Uncertainty Settings -----------------------------------
        self.sensor_std = sensor_std

        # self.UWB_RANGE_NOISE = 0.257  # Meters
        # self.UWB_RANGE_NOISE = 0.15  # Meters
        # self.UWB_RANGE_VAR = self.UWB_RANGE_NOISE ** 2
        # -----------------------------------

        self.x = np.zeros(self.NX)
        self.P = np.eye(self.NX)
        self.nis = 0

        self.state_predictor = StatePredictor(self.NX, self.N_SIGMA, self.N_AUGMENTED, self.SPEED_NOISE_VAR,
                                              self.YAW_RATE_NOISE_VAR, self.SCALE, self.WEIGHTS_M, self.WEIGHTS_C)

        self.measurement_predictor = MeasurementPredictor(sensor_std, self.N_SIGMA, self.WEIGHTS_M, self.WEIGHTS_C)

        self.state_updater = StateUpdater(self.NX, self.N_SIGMA, self.WEIGHTS_M, self.WEIGHTS_C)

    def initialize(self, x, initial_p, timestamp):
        self.x[:x.size] = x
        self.P = initial_p
        self.initialized = True
        self.timestamp = timestamp

    def update(self, data):
        if self.initialized:
            self.process(data)
        else:
            self.initialize(data.measurement_data, np.eye(self.NX), data.timestamp)

    def process(self, data):
        dt = (data.timestamp - self.timestamp) / 1e9  # seconds
        # dt = (data.timestamp - self.timestamp)  # seconds

        if dt < 0.0001:
            dt = 0.0001

        # STATE PREDICTION
        # get predicted state and covariance of predicted state, predicted sigma points in state space
        self.state_predictor.process(self.x, self.P, dt)
        self.x = self.state_predictor.x
        self.P = self.state_predictor.P
        sigma_x = self.state_predictor.sigma

        # MEASUREMENT PREDICTION
        # get predicted measurement, covariance of predicted measurement, predicted sigma points in measurement space
        self.measurement_predictor.process(sigma_x, data)
        predicted_z = self.measurement_predictor.z
        S = self.measurement_predictor.S
        sigma_z = self.measurement_predictor.sigma_z

        # STATE UPDATE
        # updated the state and covariance of state... also get the nis
        self.state_updater.process(self.x, predicted_z, data.measurement_data, S, self.P, sigma_x, sigma_z,
                                   data.data_type)
        self.x = self.state_updater.x
        self.P = self.state_updater.P
        self.nis = self.state_updater.nis

        self.timestamp = data.timestamp
