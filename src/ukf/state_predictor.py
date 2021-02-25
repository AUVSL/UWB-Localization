import numpy as np

from ukf.state import UKFState
from ukf.util import normalize


class StatePredictor:
    def __init__(self, NX, N_SIGMA, N_AUGMENTED, VAR_SPEED_NOISE, VAR_YAW_RATE_NOISE, SCALE, WEIGHTS):
        self.WEIGHTS = WEIGHTS
        self.NX = NX
        self.SCALE = SCALE
        self.N_SIGMA = N_SIGMA
        self.N_AUGMENTED = N_AUGMENTED
        self.VAR_SPEED_NOISE = VAR_SPEED_NOISE
        self.VAR_YAW_RATE_NOISE = VAR_YAW_RATE_NOISE

        self.sigma = np.zeros((NX, N_SIGMA))
        self.x = np.zeros(NX)
        self.P = np.zeros((NX, N_SIGMA))

        self.YAW_RATE_THRESHOLD = 0.001

    def compute_augmented_sigma(self, x, P):
        augmented_x = np.zeros(self.N_AUGMENTED)
        augmented_P = np.zeros((self.N_AUGMENTED, self.N_AUGMENTED))

        augmented_x[:self.NX] = x
        augmented_P[:self.NX, :self.NX] = P
        augmented_P[self.NX, self.NX] = self.VAR_SPEED_NOISE
        augmented_P[self.NX + 1, self.NX + 1] = self.VAR_YAW_RATE_NOISE

        L = np.linalg.cholesky(augmented_P)
        augmented_sigma = np.repeat(augmented_x[None], self.N_SIGMA, axis=0).T

        scaled_L = self.SCALE * L

        augmented_sigma[:, 1:self.N_AUGMENTED + 1] += scaled_L
        augmented_sigma[:, self.N_AUGMENTED + 1:] -= scaled_L

        return augmented_sigma

    def predict_sigma(self, augmented_sigma, dt):
        predicted_sigma = np.zeros((self.NX, self.N_SIGMA))

        px = augmented_sigma[UKFState.X]
        py = augmented_sigma[UKFState.Y]
        pz = augmented_sigma[UKFState.Z]
        speed = augmented_sigma[UKFState.V]
        yaw = augmented_sigma[UKFState.YAW]
        yaw_rate = augmented_sigma[UKFState.YAW_RATE]
        speed_noise = augmented_sigma[UKFState.LIN_ACCEL]
        yaw_rate_noise = augmented_sigma[UKFState.YAW_ACCEL]

        # PREDICT NEXT STEP USING CTRV Model

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        dt_2 = dt * dt

        # Acceleration noise
        p_noise = 0.5 * speed_noise * dt_2
        y_noise = 0.5 * yaw_rate_noise * dt_2

        # Velocity change
        d_yaw = yaw_rate * dt
        d_speed = speed * dt

        # Predicted speed = constant speed + acceleration noise
        p_speed = speed + speed_noise * dt

        # Predicted yaw
        p_yaw = yaw + d_yaw + y_noise

        # Predicted yaw rate
        p_yaw_rate = yaw_rate + yaw_rate_noise * dt

        mask = abs(yaw_rate) <= self.YAW_RATE_THRESHOLD
        mask_n = np.logical_not(mask)

        p_px = np.empty(self.N_SIGMA)
        p_py = np.empty(self.N_SIGMA)

        p_px[mask] = px[mask] + d_speed[mask] * cos_yaw[mask] + p_noise[mask] * cos_yaw[mask]
        p_py[mask] = py[mask] + d_speed[mask] * sin_yaw[mask] + p_noise[mask] * sin_yaw[mask]

        k = speed[mask_n] / yaw_rate[mask_n]
        theta = yaw[mask_n] + d_yaw[mask_n]
        p_px[mask_n] = px[mask_n] + k * (np.sin(theta) - sin_yaw[mask_n]) + p_noise[mask_n] * cos_yaw[mask_n]
        p_py[mask_n] = py[mask_n] + k * (cos_yaw[mask_n] - np.cos(theta)) + p_noise[mask_n] * sin_yaw[mask_n]

        predicted_sigma[UKFState.X] = p_px
        predicted_sigma[UKFState.Y] = p_py
        predicted_sigma[UKFState.Z] = pz
        predicted_sigma[UKFState.V] = p_speed
        predicted_sigma[UKFState.YAW] = p_yaw
        predicted_sigma[UKFState.YAW_RATE] = p_yaw_rate

        # ------------------

        return predicted_sigma

    def predict_x(self, predicted_sigma):
        return np.dot(predicted_sigma, self.WEIGHTS)

    def predict_P(self, predicted_sigma, predicted_x):
        sub = np.subtract(predicted_sigma.T, predicted_x).T

        normalize(sub, UKFState.YAW)

        return np.matmul(self.WEIGHTS * sub, sub.T)

    def process(self, x, P, dt):
        augmented_sigma = self.compute_augmented_sigma(x, P)
        self.sigma = self.predict_sigma(augmented_sigma, dt)
        self.x = self.predict_x(self.sigma)

        self.P = self.predict_P(self.sigma, self.x)
