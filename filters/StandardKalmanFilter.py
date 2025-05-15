"""
Mathematical highlights:
State: [x, y, vx, vy]
Prediction step:
    2. x' = A x + B x  (predict next state)
    3. P' = A P Aᵀ + Q (predict next uncertainty)
Update step:
    y = z - Cx      (innovation)
    S = C P Cᵀ + R  (innovation covariance)
    4. K = P Cᵀ S⁻¹    (Kalman gain)
    5. x = x + K y     (updated state)
    6. P = (I - KC)P   (updated uncertainty)
"""


import numpy as np

class StandardKalmanFilter:
    def __init__(self, init_state, init_cov, process_noise, measurement_noise):

        self.x = np.array(init_state, dtype=float)          # Current state vector
        self.P = np.array(init_cov, dtype=float)            # Covariance matrix
        self.Q = np.array(process_noise, dtype=float)       # Process noise
        self.R = np.array(measurement_noise, dtype=float)   # Measurement noise
        self.C = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])  # Measurement model: we observe x and y

    def predict(self, dt, u=None):
        # Prediction step
        A = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

        if u is not None:
            # B matrix adjusts prediction based on control input (acceleration)
            B = np.array([[0.5 * dt ** 2, 0],
                          [0, 0.5 * dt ** 2],
                          [dt, 0],
                          [0, dt]])
            self.x = A @ self.x + B @ u
        else:
            self.x = A @ self.x
        self.P = A @ self.P @ A.T + self.Q

    def update(self, z):
        # Update step
        z = np.array(z)
        y = z - self.C @ self.x
        S = self.C @ self.P @ self.C.T + self.R
        K = self.P @ self.C.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(len(self.P)) - K @ self.C) @ self.P

    def get_state(self):
        return self.x

    def reset(self, init_state):
        self.x = np.array(init_state)
        self.P = np.eye(len(self.x))  # or keep original init_cov if you prefer
