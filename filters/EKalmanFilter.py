import numpy as np
import math


def wrap_angle(angle):
    """Wrap angle to be between -pi and pi."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class EKalmanFilter:
    def __init__(self, init_state, init_cov, process_noise, measurement_noise):
        """
        Args:
            init_state: Initial state [x, y, theta].
            init_cov: Initial covariance matrix (3x3).
            process_noise: Process noise covariance (Q, 3x3).
                           (Increase the third component if turning is off.)
            measurement_noise: Measurement noise covariance (R, 2x2) for [range, bearing].
        """
        self.x = np.array(init_state, dtype=float)
        self.P = np.array(init_cov, dtype=float)
        self.Q = np.array(process_noise, dtype=float)
        self.R = np.array(measurement_noise, dtype=float)

    def predict(self, control, dt):
        """
        Predict the state with the motion model.

        Motion model (nonlinear, Euler integration):
          x'     = x + v*cos(theta)*dt
          y'     = y + v*sin(theta)*dt
          theta' = theta + omega*dt

        Args:
            control: Tuple (v, omega) where v is linear velocity and omega angular velocity.
            dt: Time step.
        """
        v, omega = control
        theta = self.x[2]

        # Non linear motion model
        new_x = self.x[0] + v * math.cos(theta) * dt
        new_y = self.x[1] + v * math.sin(theta) * dt
        new_theta = wrap_angle(self.x[2] + omega * dt)
        self.x = np.array([new_x, new_y, new_theta])

        # Jacobian of the motion model with respect to state:
        F = np.array([[1, 0, -v * math.sin(theta) * dt],
                      [0, 1, v * math.cos(theta) * dt],
                      [0, 0, 1]])
        self.P = F.dot(self.P).dot(F.T) + self.Q

        return self.x

    def update(self, z, landmark):
        """
        Update with a landmark measurement.

        The measurement model is:
           r = sqrt((lx - x)^2 + (ly - y)^2)
           bearing = wrap_angle(atan2(ly - y, lx - x) - theta)

        Args:
            z: Measurement vector [range, bearing] (with noise).
            landmark: (lx, ly) corresponding to the feature. Here, this is
                      the midpoint of the wall segment.
        """
        lx, ly = landmark
        x, y, theta = self.x
        dx = lx - x
        dy = ly - y
        q = dx ** 2 + dy ** 2
        sqrt_q = math.sqrt(q)

        # Predicted measurement
        z_pred = np.array([sqrt_q, wrap_angle(math.atan2(dy, dx) - theta)])

        # Innovation
        y_innov = np.array(z) - z_pred
        y_innov[1] = wrap_angle(y_innov[1])

        # Jacobian H of the measurement model with respect to state:
        H = np.array([
            [-dx / sqrt_q, -dy / sqrt_q, 0],
            [dy / q, -dx / q, -1]
        ])

        S = H.dot(self.P).dot(H.T) + self.R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))

        self.x = self.x + K.dot(y_innov)
        self.x[2] = wrap_angle(self.x[2])

        I = np.eye(3)
        self.P = (I - K.dot(H)).dot(self.P)
        return self.x

    def get_state(self):
        return self.x

    def get_covariance(self):
        return self.P
