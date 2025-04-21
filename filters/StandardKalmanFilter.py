"""
StandardKalmanFilter.py
------------------------
This class implements a basic 2D constant velocity Kalman Filter for tracking a robot's position.

Mathematical highlights:
- State: [x, y, vx, vy]
- Prediction step:
    x' = F x        (predict next state)
    P' = F P Fᵀ + Q (predict next uncertainty)
- Update step:
    y = z - Hx      (innovation)
    S = H P Hᵀ + R  (innovation covariance)
    K = P Hᵀ S⁻¹    (Kalman gain)
    x = x + K y     (updated state)
    P = (I - KH)P   (updated uncertainty)
"""


import numpy as np

class StandardKalmanFilter:
    def __init__(self, init_state, init_cov, process_noise, measurement_noise):

        self.x = np.array(init_state, dtype=float)          # Current state vector
        self.P = np.array(init_cov, dtype=float)            # Covariance matrix
        self.Q = np.array(process_noise, dtype=float)       # Process noise
        self.R = np.array(measurement_noise, dtype=float)   # Measurement noise
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])  # Measurement model: we observe x and y

    def predict(self, dt):
        # 🔁 Prediction step
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

        self.x = F @ self.x                     # Predict state
        self.P = F @ self.P @ F.T + self.Q      # Predict uncertainty

    def update(self, z):
        # 📥 Update step
        z = np.array(z)
        y = z - self.H @ self.x                     # Innovation (residual)
        S = self.H @ self.P @ self.H.T + self.R     # Innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)    # Kalman Gain

        self.x = self.x + K @ y
        self.P = (np.eye(len(self.P)) - K @ self.H) @ self.P    # Update uncertainty

    def get_state(self):
        return self.x
