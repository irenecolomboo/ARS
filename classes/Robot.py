import pygame
import math
from classes.Sensors import Sensors
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

    def reset(self):
        self.position = [400, 300]
        self.angle = 0

        self.V_l = 0
        self.V_r = 0

        self.sensors.update(self.environment.get_walls())


