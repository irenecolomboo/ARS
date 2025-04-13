import pygame
import math
import numpy as np
from classes.Sensors import Sensors
from filters.KalmanFilter import KalmanFilter, wrap_angle


class Robot:
    def __init__(self, position, radius=25, angle=0, environment=None, collision_handler=None):
        self.environment = environment
        self.collision_handler = collision_handler

        self.position = list(position)
        self.radius = radius
        self.angle = angle

        # Differential drive: wheel speeds and parameters.
        self.V_l = 0
        self.V_r = 0
        self.wheel_base = 60
        self.max_speed = 5
        self.speed_increment = 0.1

        self.sensors = Sensors(self)  # the sensor code provided externally.

        # Initialize EKF with the robot's state.
        init_state = [self.position[0], self.position[1], self.angle]
        init_cov = np.diag([10, 10, 0.1])
        # Process noise: note a slight increase in rotational noise to account for turning imperfections.
        process_noise = np.diag([0.5, 0.5, 0.1])
        # Measurement noise based on sensor quality.
        measurement_noise = np.diag([0.5 ** 2, (math.radians(5)) ** 2])
        self.kf = KalmanFilter(init_state, init_cov, process_noise, measurement_noise)

        # For visualizing trajectories.
        self.true_trajectory = [tuple(self.position)]
        self.estimated_trajectory = [tuple(init_state[:2])]

        # Sensor range (for both sensor drawing and landmark update)
        self.sensor_range = 200

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
            # Optional friction for smooth behavior.
            friction = 0.05
            if abs(self.V_l) > friction:
                self.V_l -= friction * (1 if self.V_l > 0 else -1)
            else:
                self.V_l = 0

            if abs(self.V_r) > friction:
                self.V_r -= friction * (1 if self.V_r > 0 else -1)
            else:
                self.V_r = 0

        # Corrected Left / Right:
        if keys[pygame.K_LEFT]:
            # For a left turn: increase left wheel speed and decrease right wheel speed.
            self.V_l = min(self.V_l + increment, self.max_speed)
            self.V_r = max(self.V_r - increment, -self.max_speed)
        elif keys[pygame.K_RIGHT]:
            # For a right turn: decrease left wheel speed and increase right wheel speed.
            self.V_l = max(self.V_l - increment, -self.max_speed)
            self.V_r = min(self.V_r + increment, self.max_speed)

    def update(self, dt=1):
        # Get control inputs from wheel speeds (differential drive kinematics).
        v = (self.V_r + self.V_l) / 2
        omega = (self.V_r - self.V_l) / self.wheel_base

        # Compute proposed new pose.
        proposed_angle = self.angle + omega * dt
        dx = v * math.cos(proposed_angle) * dt
        dy = v * math.sin(proposed_angle) * dt  # note: adjust sign if needed for your coordinate system
        proposed_position = (self.position[0] + dx, self.position[1] + dy)

        # Handle collisions.
        corrected_position = self.collision_handler.handle_collision(self, proposed_position)
        self.angle = proposed_angle
        self.position = list(corrected_position)

        # Update sensors for visualization.
        self.sensors.update(self.environment.get_walls())

        # Record the true trajectory.
        self.true_trajectory.append(tuple(self.position))

        # --- EKF Prediction ---
        self.kf.predict((v, omega), dt)

        # --- EKF Update using wall landmarks ---
        # Here, each wall is treated as a landmark. We compute its midpoint.
        walls = self.environment.get_walls()
        for wall in walls:
            # Wall is assumed to be represented as (x1, y1, x2, y2)
            lm = ((wall[0] + wall[2]) / 2.0, (wall[1] + wall[3]) / 2.0)
            # Check if the landmark is within sensor range.
            dx_lm = lm[0] - self.position[0]
            dy_lm = lm[1] - self.position[1]
            distance = math.hypot(dx_lm, dy_lm)
            if distance <= self.sensor_range:
                # Simulate measurement noise.
                range_noise = np.random.normal(0, 0.5)
                bearing_noise = np.random.normal(0, math.radians(5))
                measured_range = distance + range_noise
                # True bearing based on robot's true angle.
                true_bearing = wrap_angle(math.atan2(dy_lm, dx_lm) - self.angle)
                measured_bearing = wrap_angle(true_bearing + bearing_noise)
                z = [measured_range, measured_bearing]
                self.kf.update(z, lm)

        estimated_state = self.kf.get_state()
        self.estimated_trajectory.append(tuple(estimated_state[:2]))

    def draw(self, screen):
        # Draw the actual robot.
        pygame.draw.circle(screen, (255, 100, 50),
                           (int(self.position[0]), int(self.position[1])),
                           self.radius)
        line_length = self.radius
        end_x = self.position[0] + line_length * math.cos(self.angle)
        end_y = self.position[1] + line_length * math.sin(self.angle)
        pygame.draw.line(screen, (0, 0, 0),
                         (self.position[0], self.position[1]),
                         (end_x, end_y), 3)

        font = pygame.font.SysFont(None, 20)
        text_left = font.render(f"V_l: {self.V_l:.1f}", True, (0, 0, 0))
        text_right = font.render(f"V_r: {self.V_r:.1f}", True, (0, 0, 0))
        screen.blit(text_left, (30, 30))
        screen.blit(text_right, (30, 50))

        # Draw the sensor lines.
        self.sensors.draw(screen)

        # --- Draw Walls as Landmarks ---
        walls = self.environment.get_walls()
        for wall in walls:
            # Draw the wall (if not already drawn by the environment).
            pygame.draw.line(screen, (0, 0, 0), (wall[0], wall[1]), (wall[2], wall[3]), 3)
            # Compute the landmark (midpoint)
            lm = ((wall[0] + wall[2]) / 2.0, (wall[1] + wall[3]) / 2.0)
            pygame.draw.circle(screen, (0, 0, 0), (int(lm[0]), int(lm[1])), 5)

        # --- Draw Actual Trajectory (solid red line) ---
        if len(self.true_trajectory) > 1:
            pygame.draw.lines(screen, (255, 0, 0), False, self.true_trajectory, 2)

        # --- Draw Estimated Trajectory (blue dotted points) ---
        if len(self.estimated_trajectory) > 1:
            for pt in self.estimated_trajectory:
                pygame.draw.circle(screen, (0, 0, 255), (int(pt[0]), int(pt[1])), 2)

        # --- Draw Covariance Ellipse from the EKF ---
        self.draw_covariance_ellipse(screen, self.kf.get_state(), self.kf.get_covariance())

        # Display the EKF estimated state.
        kf_state = self.kf.get_state()
        text_filter = font.render(
            f"KF: ({kf_state[0]:.1f}, {kf_state[1]:.1f}, {kf_state[2]:.1f})", True, (0, 0, 255)
        )
        screen.blit(text_filter, (30, 70))

    def draw_covariance_ellipse(self, screen, state, covariance, nstd=2, color=(0, 0, 255)):
        """
        Draw an ellipse representing the uncertainty in x-y.
        nstd: Number of standard deviations (e.g., 2 ≈ 95% confidence).
        """
        cov_xy = covariance[0:2, 0:2]
        eig_vals, eig_vecs = np.linalg.eig(cov_xy)
        angle = math.atan2(eig_vecs[1, 0], eig_vecs[0, 0])
        angle_deg = math.degrees(angle)
        width = 2 * nstd * math.sqrt(eig_vals[0])
        height = 2 * nstd * math.sqrt(eig_vals[1])

        ellipse_rect = pygame.Rect(0, 0, width, height)
        ellipse_rect.center = (state[0], state[1])

        ellipse_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        pygame.draw.ellipse(ellipse_surface, color, ellipse_surface.get_rect(), 2)
        rotated_surface = pygame.transform.rotate(ellipse_surface, -angle_deg)
        rot_rect = rotated_surface.get_rect(center=(state[0], state[1]))
        screen.blit(rotated_surface, rot_rect)

    def reset(self):
        self.position = [400, 300]
        self.angle = 0
        self.V_l = 0
        self.V_r = 0
        self.sensors.update(self.environment.get_walls())

        self.true_trajectory = [tuple(self.position)]
        self.estimated_trajectory = [(self.position[0], self.position[1])]

        self.kf.x = np.array([self.position[0], self.position[1], self.angle])
