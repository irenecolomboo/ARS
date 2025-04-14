"""
--------
This class represents a mobile robot simulation using Pygame. It includes:
- Robot motion with differential wheel speeds.
- Simulated sensors.
- Collision handling.
- Kalman Filter (KF) for noisy position estimation.
- Real-time visualization of both true and estimated trajectories.

Mathematical highlights:
- Motion model: differential drive (based on wheel speeds).
- Kalman Filter steps:
    - Prediction: Uses constant velocity model (x, y, vx, vy).
    - Update: Combines noisy measurement with prediction to estimate position.
"""


import pygame
import math
import numpy as np
from classes.Sensors import Sensors
from filters.StandardKalmanFilter import StandardKalmanFilter  # If in separate file
class Robot:
    def __init__(self, position, radius=25, angle=0, environment=None, collision_handler=None):

        self.environment = environment
        self.collision_handler = collision_handler

        self.position = list(position)
        self.radius = radius
        self.angle = angle

        self.velocity = 0
        self.angular_velocity = 0
        self.speed = 2
        self.rotation_speed = math.radians(5)

        # Robot's wheel speeds
        self.V_l = 0 
        self.V_r = 0 
        self.wheel_base = 60  # Distance between wheels

        self.max_speed = 5  # Max wheel speed
        self.speed_increment = 0.1  # Small increment

        self.sensors = Sensors(self)

        # ➕ Kalman Filter Initialization (mathematical models: constant velocity)
        self.kf = StandardKalmanFilter(
            init_state=[self.position[0], self.position[1], 0, 0],  # x, y, vx, vy
            init_cov=np.eye(4) * 10,                                # Initial uncertainty
            process_noise=np.eye(4) * 1,                            # Q: process noise
            measurement_noise=np.eye(2) * 10                        # R: measurement noise
        )
        self.kf_trajectory = []
        self.true_trajectory = []


    def handle_keys(self):
        keys = pygame.key.get_pressed()
        increment = self.speed_increment

        # Forward / Backward
        if keys[pygame.K_UP]:
            self.V_l = min(self.V_l + increment, self.max_speed)
            self.V_r = min(self.V_r + increment, self.max_speed)
        elif keys[pygame.K_DOWN]:
            self.V_l = max(self.V_l - increment, -self.max_speed)
            self.V_r = max(self.V_r - increment, -self.max_speed)
        else:
            # Optional friction (keep nice behaviour)
            friction = 0.05
            if abs(self.V_l) > friction:
                self.V_l -= friction * (1 if self.V_l > 0 else -1)
            else:
                self.V_l = 0

            if abs(self.V_r) > friction:
                self.V_r -= friction * (1 if self.V_r > 0 else -1)
            else:
                self.V_r = 0

        # Left / Right
        if keys[pygame.K_LEFT]:
            self.V_l = max(self.V_l - increment, -self.max_speed)
            self.V_r = min(self.V_r + increment, self.max_speed)
        elif keys[pygame.K_RIGHT]:
            self.V_l = min(self.V_l + increment, self.max_speed)
            self.V_r = max(self.V_r - increment, -self.max_speed)


    def update(self, dt=1):
        v = (self.V_r + self.V_l) / 2
        omega = (self.V_r - self.V_l) / self.wheel_base

        # new position
        proposed_angle = self.angle + omega * dt
        dx = v * math.cos(proposed_angle) * dt
        dy = -v * math.sin(proposed_angle) * dt
        proposed_position = (self.position[0] + dx, self.position[1] + dy)

        # Check collisions
        corrected_position = self.collision_handler.handle_collision(self, proposed_position)

        self.angle = proposed_angle
        self.position = list(corrected_position)

        self.sensors.update(self.environment.get_walls())

        # ➕ Kalman Filter steps (mathematical operations)
        self.kf.predict(dt)                     # 🔁 Predict step: x' = F x  |  P' = FPFᵀ + Q
        # Use the noisy actual position as a measurement
        self.kf.update(self.position)
        # Simulated noisy position measurement
        noisy_x = self.position[0] + np.random.normal(0, 20)
        noisy_y = self.position[1] + np.random.normal(0, 20)
        self.kf.update([noisy_x, noisy_y])       # 📥 Update step: combine prediction and measurement

        self.true_trajectory.append(tuple(self.position))
        self.kf_trajectory.append(tuple(self.kf.x[:2]))

        max_length = 500
        if len(self.true_trajectory) > max_length:
            self.true_trajectory.pop(0)
            self.kf_trajectory.pop(0)


    def draw(self, screen):
        pygame.draw.circle(screen, (255, 100, 50), (int(self.position[0]), int(self.position[1])), self.radius)

        line_length = self.radius
        end_x = self.position[0] + line_length * math.cos(self.angle)
        end_y = self.position[1] - line_length * math.sin(self.angle)
        pygame.draw.line(screen, (0, 0, 0), (self.position[0], self.position[1]), (end_x, end_y), 3)

        font = pygame.font.SysFont(None, 20)
        text_left = font.render(f"V_l: {self.V_l:.1f}", True, (0, 0, 0))
        text_right = font.render(f"V_r: {self.V_r:.1f}", True, (0, 0, 0))
        screen.blit(text_left, (30, 30))
        screen.blit(text_right, (30, 50))

        self.sensors.draw(screen)

        # ➕ Visualize KF estimated position
        estimated_state = self.kf.get_state()
        est_x, est_y = int(estimated_state[0]), int(estimated_state[1])
        pygame.draw.circle(screen, (50, 255, 50), (est_x, est_y), 8)  # Green dot
        font = pygame.font.SysFont(None, 20)
        screen.blit(font.render("KF", True, (0, 100, 0)), (est_x + 10, est_y - 10))

        # Draw actual trajectory (blue)
        if len(self.true_trajectory) > 1:
            pygame.draw.lines(screen, (0, 0, 255), False, self.true_trajectory, 2)

        # Draw KF-estimated trajectory (green)
        if len(self.kf_trajectory) > 1:
            pygame.draw.lines(screen, (0, 255, 0), False, self.kf_trajectory, 2)


    def reset(self):
        self.position = [400, 300]
        self.angle = 0

        self.V_l = 0
        self.V_r = 0

        self.sensors.update(self.environment.get_walls())


