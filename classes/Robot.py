import pygame
import math
from classes.Sensors import Sensors
import numpy as np
from filters.StandardKalmanFilter import StandardKalmanFilter
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

        self.sensors = Sensors(self,num_sensors=12)

        self.estimate_trajectory = []

        # Kalman Filter
        self.init_cov = np.eye(4) * 1.0  # Initial uncertainty
        self.process_noise = np.eye(4) * 0.1  # Assumes small process noise
        self.measurement_noise = np.eye(2) * 1.0  # Measurement noise

        self.estimated_state = np.eye(2) * 1.0
        self.trajectory = []
        self.kf = StandardKalmanFilter(init_state=self.position + [0,0],init_cov=self.init_cov,process_noise=self.process_noise,measurement_noise=self.measurement_noise)

        # --- Uncertainty circle attributes ---
        self.min_uncertainty = 10  # Minimum uncertainty radius
        self.max_uncertainty = 100  # Maximum uncertainty radius
        self.uncertainty_growth_rate = 2  # Growth rate per update (you can adjust this)
        self.uncertainty_radius = self.min_uncertainty

        self.original = pygame.image.load("plane1.png").convert_alpha()
        self.original.set_colorkey((255, 255, 255))
        self.image = pygame.transform.rotozoom(self.original, -45, 0.08)

        self.road = pygame.image.load("road.jpg")
        self.road = pygame.transform.rotozoom(self.road, 0, 0.15)



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

        # Kalman filter
        current_vx = self.kf.x[2]
        current_vy = self.kf.x[3]
        ax_cmd = (dx - current_vx) / dt
        ay_cmd = (dy - current_vy) / dt
        control_input = np.array([ax_cmd, ay_cmd])
        #print(control_input)

        self.kf.predict(dt, control_input)
        # 🟡 Simulated noisy measurement (from a sensor, e.g., GPS)
        true_pos = self.position  # e.g., [x_true, y_true]
        landmarks = self.environment.get_landmarks()
        distances = []
        bearings = []
        copy = landmarks.copy()
        for landmark in landmarks:
            r, phi = self.get_distance_and_bearing(true_pos, self.angle, landmark)
            #print(r)
            if r > self.sensors.max_distance:
                if self.sensors.check_if_wall(phi, true_pos, self.environment.get_walls()):
                    copy.remove(landmark)
            # if self.sensors.check_if_wall(self.angle,true_pos,self.environment.get_walls()):
            #     continue
            else:
                distances.append(r)
                bearings.append(phi)

        if(len(distances) > 1): # Can be only 2 for the bearing one but need to be at least 3 for the triangulation
            #pos = -(self.trilateration(copy, distances))
            pos = (self.triangulate_with_bearing(copy, distances, bearings))

            # Reset uncertainty since we got valid sensor data
            self.uncertainty_radius = self.min_uncertainty
        else:
            self.uncertainty_radius = min(self.uncertainty_radius + (self.V_l + self.V_r)/4, self.max_uncertainty)
            pos = self.kf.get_state()[:2]

        #print(true_pos)
        #print(pos)
        #orientation = self.estimate_orientation(pos, landmarks, bearings)


        noisy_measurement = pos + np.random.normal(0, 1.0, size=2)

        # 🔵 Update with measurement
        self.kf.update(noisy_measurement)

        # 🔴 Get the estimated state
        estimated_state = self.kf.get_state()
        #print(estimated_state)

        self.estimate_trajectory.append(estimated_state[:2])


    def get_distance_and_bearing(self, robot_pose, degree, landmark_pos):
        x, y = robot_pose
        theta_deg = degree
        lx, ly = landmark_pos

        dx, dy = lx - x, ly - y
        r = np.sqrt(dx ** 2 + dy ** 2)

        # World-frame angle to landmark
        angle_world = np.arctan2(dy, dx)
        theta_rad = np.deg2rad(theta_deg)

        # Relative angle in robot frame
        phi = angle_world - theta_rad

        # Normalize to [-pi, pi]
        phi = (phi + np.pi) % (2 * np.pi) - np.pi

        return r, np.rad2deg(phi)  # return bearing in degrees

    def trilateration(self, landmarks, distances): # Without bearing
        """
        Estimate position from 3 or more landmarks and distances using least squares trilateration.
        """
        A = []
        b = []
        for i in range(1, len(landmarks)):
            x0, y0 = landmarks[0]
            xi, yi = landmarks[i]
            ri2 = distances[i] ** 2
            r02 = distances[0] ** 2
            A.append([2 * (xi - x0), 2 * (yi - y0)])
            b.append([ri2 - r02 - xi ** 2 - yi ** 2 + x0 ** 2 + y0 ** 2])
        A = np.array(A)
        b = np.array(b)

        pos, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        return pos.flatten()

    import numpy as np

    def triangulate_with_bearing(self, landmarks, distances, bearings, bearing_idx=0):
        if len(landmarks) < 2:
            raise ValueError("At least two landmarks are required for triangulation.")

        if not (0 <= bearing_idx < len(landmarks)):
            raise ValueError("bearing_idx must be within the range of provided landmarks.")

        # Get the reference landmark data (the one with the bearing measurement):
        x_ref, y_ref = landmarks[bearing_idx]
        r_ref = distances[bearing_idx]
        # Determine the bearing: if a single angle is provided, use that; otherwise, select from the list.
        if len(bearings) == 1:
            bearing_deg = bearings[0]
        else:
            bearing_deg = bearings[bearing_idx]
        bearing_rad = np.radians(bearing_deg)

        # Build the system of equations.
        A_list = []
        b_list = []

        # Equation 1: Bearing constraint from the reference landmark.
        A_list.append([np.sin(bearing_rad), -np.cos(bearing_rad)])
        b_list.append(np.sin(bearing_rad) * x_ref - np.cos(bearing_rad) * y_ref)

        # Equation 2: For each other landmark, add the distance difference equation.
        for j in range(len(landmarks)):
            if j == bearing_idx:
                continue  # Skip the reference landmark.
            xj, yj = landmarks[j]
            rj = distances[j]
            # The equation: 2*(xj - x_ref)*x + 2*(yj - y_ref)*y =
            #             (xj² + yj² - x_ref² - y_ref²) - (rj² - r_ref²)
            A_list.append([2 * (xj - x_ref), 2 * (yj - y_ref)])
            b_list.append((xj**2 + yj**2 - x_ref**2 - y_ref**2) - (rj**2 - r_ref**2))

        # Convert lists to numpy arrays.
        A = np.array(A_list)
        b = np.array(b_list)

        # If only two equations are present, try an exact solve; otherwise, use least squares.
        if A.shape[0] == 2:
            try:
                pos = np.linalg.solve(A, b)
            except np.linalg.LinAlgError:
                pos, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        else:
            pos, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

        return pos

    def estimate_orientation(self, estimated_pos, landmarks, measured_bearings):
        """
        Estimate orientation θ of robot from bearings to landmarks.
        measured_bearings: angles in robot frame (Φ)
        """
        angles = []
        x, y = estimated_pos
        for (lx, ly), phi in zip(landmarks, measured_bearings):
            dx, dy = lx - x, ly - y
            bearing_global = np.arctan2(dy, dx)  # angle in world frame
            theta = (bearing_global - np.deg2rad(phi)) % (2 * np.pi)
            angles.append(theta)
        theta_avg = np.mean(angles)
        return np.rad2deg(theta_avg) % 360

    def draw(self, screen):
        #pygame.draw.circle(screen, (255, 100, 50), (int(self.position[0]), int(self.position[1])), self.radius)

        # Rotate the image by converting angle from radians to degrees.
        rotated_image = pygame.transform.rotate(self.image, math.degrees(self.angle))
        # Update the rect so the rotated image is centered at the position.
        new_rect = rotated_image.get_rect(center=(self.position[0], self.position[1]))
        screen.blit(rotated_image, new_rect.topleft)

        # Make road
        total = 45
        for i in range(7):
            new_rect2 = rotated_image.get_rect(center=(total, 185))
            screen.blit(self.road, new_rect2)
            total = total + 120
        total = 45
        for i in range(7):
            new_rect2 = rotated_image.get_rect(center=(total, 320))
            screen.blit(self.road, new_rect2)
            total = total + 120

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

        # Draw trajectory
        if len(self.trajectory) > 1:
            pygame.draw.lines(screen, (0, 255, 0), False, self.trajectory, 2)

        # Draw estimated trajectory (green)
        if len(self.estimate_trajectory) > 1:
            pygame.draw.lines(screen, (0, 255, 0), False, self.estimate_trajectory, 2)

        pygame.draw.circle(
            screen,
            (0, 0, 255),  # blue color for uncertainty
            (int(self.position[0]), int(self.position[1])),
            int(self.uncertainty_radius),
            2  # thickness of 2 pixels
        )

    def reset(self):
        self.position = [400, 300]
        self.angle = 0

        self.V_l = 0
        self.V_r = 0

        self.sensors.update(self.environment.get_walls())


