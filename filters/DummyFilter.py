# File: classes/DummyFilter.py
import math


class DummyFilter:
    """
    A simple dummy filter for 2D robot localization.

    The state is represented as [x, y, theta]. The filter uses a basic motion
    prediction model to update the state. The correction step is a stub that
    just logs the received measurement.
    """

    def __init__(self, init_state=None):
        if init_state is None:
            init_state = [0.0, 0.0, 0.0]
        self.state = init_state

    def predict(self, control, dt):
        """
        Updates the state using a basic motion model.

        Args:
            control (tuple): (v, omega) where v is the linear velocity and omega is the angular velocity.
            dt (float): Time step duration.

        Returns:
            list: Updated state [x, y, theta]
        """
        v, omega = control
        x, y, theta = self.state

        # Update theta and normalize
        new_theta = theta + omega * dt
        new_theta = (new_theta + math.pi) % (2 * math.pi) - math.pi

        # Predict new position based on the new heading
        new_x = x + v * dt * math.cos(new_theta)
        new_y = y + v * dt * math.sin(new_theta)

        self.state = [new_x, new_y, new_theta]
        return self.state

    def correct(self, measurement):
        """
        A dummy measurement correction step. In this filter, the measurement
        is logged but the state remains unchanged.

        Args:
            measurement (any): Sensor measurement data.

        Returns:
            list: The unchanged state
        """
        print("[DummyFilter] Received measurement:", measurement)
        return self.state

    def get_state(self):
        """
        Returns the current filter state.
        """
        return self.state
