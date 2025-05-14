import pygame
import math
from classes.Sensors import Sensors
import numpy as np
from filters.StandardKalmanFilter import StandardKalmanFilter
from classes.OccupancyGridMap import OccupancyGridMap  # ensure this import is present

class Robot:
    def __init__(self, position, radius=25, angle=0, environment=None, collision_handler=None):

        self.environment = environment
        self.collision_handler = collision_handler

        self.position = list(position)
        self.start_position = list(position)  # Save the initial starting position
        self.start_angle = angle              # Save initial orientation    

        self.previousposition = list(position)
        self.radius = radius
        self.angle = angle

        self.velocity = 0
        self.angular_velocity = 0
        self.speed = 1
        self.rotation_speed = math.radians(5)

        # Robot's wheel speeds
        self.V_l = 0
        self.V_r = 0
        self.wheel_base = 60  # Distance between wheels

        self.max_speed = 5  # Max wheel speed
        self.speed_increment = 0.1  # Small increment

        self.sensors = Sensors(self, max_distance=200, num_sensors=12)

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

        # In __init__(), after setting self.environment
        map_width = self.environment.width
        map_height = self.environment.height
        resolution = 4  # e.g., 4 pixels per cell, or choose what matches your screen/grid
        self.occupancy_map = OccupancyGridMap(map_width, map_height, resolution)

        self.original = pygame.image.load("plane1.png").convert_alpha()
        self.original.set_colorkey((255, 255, 255))
        self.image = pygame.transform.rotozoom(self.original, -45, 0.08)

        self.road = pygame.image.load("road.jpg")
        self.road = pygame.transform.rotozoom(self.road, 0, 0.15)

        self.roadv = pygame.image.load("road.jpg")
        self.roadv = pygame.transform.rotozoom(self.roadv, 90, 0.255)

        self.roadd = pygame.image.load("road.jpg")
        self.roadd = pygame.transform.rotozoom(self.roadd, 110, 0.22)

        self.grass = pygame.image.load("grass.jpg")
        self.grass = pygame.transform.rotozoom(self.grass, 0, 2)

        self.tower = pygame.image.load("control_tower.png")
        self.tower = pygame.transform.rotozoom(self.tower, 0, 0.2)

        self.parking = pygame.image.load("parking.png")
        self.parking = pygame.transform.rotozoom(self.parking, 0, 0.8)

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

        # Update sensor readings and scan points
        self.sensors.update(self.environment.get_walls())

        # Update occupancy map with scan points
        self.occupancy_map.update_from_scan(self.position, self.sensors.scan_points, max_range=self.sensors.max_distance)

        # Kalman filter
        current_vx = self.kf.x[2]
        current_vy = self.kf.x[3]
        ax_cmd = (dx - current_vx) / dt
        ay_cmd = (dy - current_vy) / dt
        control_input = np.array([ax_cmd, ay_cmd])

        self.kf.predict(dt, control_input)

        # Simulated noisy measurement (from a sensor, e.g., GPS)
        true_pos = self.position
        landmarks = self.environment.get_landmarks()
        distances = []
        bearings = []
        copy = landmarks.copy()

        for landmark in landmarks:
            r, phi = self.get_distance_and_bearing(true_pos, self.angle, landmark)
            if r > self.sensors.max_distance:
                copy.remove(landmark)
            else:
                distances.append(r)
                bearings.append(phi)

        if(len(distances) > 1):
            pos = self.triangulate_with_bearing(copy, distances, bearings)
            self.previousposition = pos
            self.uncertainty_radius = self.min_uncertainty
        else:
            pos = self.previousposition
            self.uncertainty_radius = min(self.uncertainty_radius + (self.V_l + self.V_r)/4, self.max_uncertainty)
            pos = self.kf.get_state()[:2]

        noisy_measurement = pos + np.random.normal(0, 1.0, size=2)
        self.kf.update(noisy_measurement)

        estimated_state = self.kf.get_state()
        self.estimate_trajectory.append(estimated_state[:2])

    def get_distance_and_bearing(self, robot_pose, degree, landmark_pos):
        x, y = robot_pose
        theta_deg = degree
        lx, ly = landmark_pos

        dx, dy = lx - x, ly - y
        r = np.sqrt(dx ** 2 + dy ** 2)
        angle_world = np.arctan2(dy, dx)
        theta_rad = np.deg2rad(theta_deg)
        phi = angle_world - theta_rad
        phi = (phi + np.pi) % (2 * np.pi) - np.pi
        return r, np.rad2deg(phi)

    def triangulate_with_bearing(self, landmarks, distances, bearings, bearing_idx=0):
        if len(landmarks) < 2:
            raise ValueError("At least two landmarks are required for triangulation.")

        if not (0 <= bearing_idx < len(landmarks)):
            raise ValueError("bearing_idx must be within the range of provided landmarks.")

        x_ref, y_ref = landmarks[bearing_idx]
        r_ref = distances[bearing_idx]
        bearing_deg = bearings[bearing_idx]
        bearing_rad = np.radians(bearing_deg)

        A_list = [[np.sin(bearing_rad), -np.cos(bearing_rad)]]
        b_list = [np.sin(bearing_rad) * x_ref - np.cos(bearing_rad) * y_ref]

        for j in range(len(landmarks)):
            if j == bearing_idx:
                continue
            xj, yj = landmarks[j]
            rj = distances[j]
            A_list.append([2 * (xj - x_ref), 2 * (yj - y_ref)])
            b_list.append((xj**2 + yj**2 - x_ref**2 - y_ref**2) - (rj**2 - r_ref**2))

        A = np.array(A_list)
        b = np.array(b_list)

        if A.shape[0] == 2:
            try:
                pos = np.linalg.solve(A, b)
            except np.linalg.LinAlgError:
                pos, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        else:
            pos, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

        return pos.flatten()

    def draw(self, screen, screen_Grid):

        # Grass
        rotated_image = pygame.transform.rotate(self.grass, 0)
        new_rect2 = rotated_image.get_rect(center=(170, 240))
        screen.blit(self.grass, new_rect2)

        # Control Tower
        rotated_image = pygame.transform.rotate(self.tower, 0)
        new_rect2 = rotated_image.get_rect(center=(375, 125))
        screen.blit(self.tower, new_rect2)
        new_rect2 = rotated_image.get_rect(center=(150, 125))
        screen.blit(self.tower, new_rect2)
        new_rect2 = rotated_image.get_rect(center=(375, 365))
        screen.blit(self.tower, new_rect2)
        new_rect2 = rotated_image.get_rect(center=(150, 365))
        screen.blit(self.tower, new_rect2)

        new_rect2 = rotated_image.get_rect(center=(660, 240))
        screen.blit(self.tower, new_rect2)
        new_rect2 = rotated_image.get_rect(center=(660, 365))
        screen.blit(self.tower, new_rect2)

        # Make road diagonal
        rotated_image = pygame.transform.rotate(self.roadd, 0)
        new_rect2 = rotated_image.get_rect(center=(270, 500))
        screen.blit(self.roadd, new_rect2)

        # Make road vertical
        rotated_image = pygame.transform.rotate(self.roadv, 0)
        total = 200
        for i in range(3):
            new_rect2 = rotated_image.get_rect(center=(250, total))
            screen.blit(self.roadv, new_rect2)
            total = total + 80
        total = 247
        for i in range(2):
            new_rect2 = rotated_image.get_rect(center=(740, total))
            screen.blit(self.roadv, new_rect2)
            total = total + 240

        # Make road horizontal
        rotated_image = pygame.transform.rotate(self.road, 0)
        total = 45
        for i in range(7):
            new_rect2 = rotated_image.get_rect(center=(total, 185))
            screen.blit(self.road, new_rect2)
            new_rect2 = rotated_image.get_rect(center=(total, 305))
            screen.blit(self.road, new_rect2)
            new_rect2 = rotated_image.get_rect(center=(total, 425))
            screen.blit(self.road, new_rect2)
            new_rect2 = rotated_image.get_rect(center=(total, 550))
            screen.blit(self.road, new_rect2)
            total = total + 120

        #
        parking_image = pygame.transform.rotate(self.parking, 0)
        parking_zone = parking_image.get_rect(center=(135, 55))
        screen.blit(self.parking, parking_zone)
        parking_zone2 = parking_image.get_rect(center=(277, 55))
        screen.blit(self.parking,parking_zone2)
        
        # Player draw
        # pygame.draw.circle(screen, (255, 100, 50), (int(self.position[0]), int(self.position[1])), self.radius)
        # Rotate the image by converting angle from radians to degrees.
        rotated_image = pygame.transform.rotate(self.image, math.degrees(self.angle))
        # Update the rect so the rotated image is centered at the position.
        new_rect = rotated_image.get_rect(center=(self.position[0], self.position[1]))
        screen.blit(rotated_image, new_rect.topleft)

        line_length = self.radius
        end_x = self.position[0] + line_length * math.cos(self.angle)
        end_y = self.position[1] - line_length * math.sin(self.angle)
        pygame.draw.line(screen, (0, 0, 0), (self.position[0], self.position[1]), (end_x, end_y), 3)

        font = pygame.font.SysFont(None, 20)
        screen.blit(font.render(f"V_l: {self.V_l:.1f}", True, (0, 0, 0)), (30, 30))
        screen.blit(font.render(f"V_r: {self.V_r:.1f}", True, (0, 0, 0)), (30, 50))

        self.sensors.draw(screen)

        self.occupancy_map.draw_map_on_screen(screen_Grid, self.trajectory, self.estimate_trajectory, scale=4)

        # if len(self.trajectory) > 1:
        #     pygame.draw.lines(screen, (0, 255, 0), False, self.trajectory, 2)
        #
        # if len(self.estimate_trajectory) > 1:
        #     pygame.draw.lines(screen, (0, 255, 0), False, self.estimate_trajectory, 2)

        pygame.draw.circle(screen, (0, 0, 255), (int(self.position[0]), int(self.position[1])), int(self.uncertainty_radius), 2)

    def reset(self):
        self.position = list(self.start_position)  # Go back to original start
        self.angle = self.start_angle

        self.V_l = 0
        self.V_r = 0

        self.estimate_trajectory = []
        self.kf.reset(self.start_position + [0, 0])  # Reset the Kalman filter if needed

        self.uncertainty_radius = self.min_uncertainty
        self.sensors.update(self.environment.get_walls())


    def follow_move_command(self, move):
        if move == "UP":
            self.V_l = self.speed
            self.V_r = self.speed
            self.angle = math.radians(90)
        elif move == "DOWN":
            self.V_l = self.speed
            self.V_r = self.speed
            self.angle = math.radians(-90)
        elif move == "LEFT":
            self.V_l = self.speed
            self.V_r = self.speed
            self.angle = math.radians(180)
        elif move == "RIGHT":
            self.V_l = self.speed
            self.V_r = self.speed
            self.angle = math.radians(0)
