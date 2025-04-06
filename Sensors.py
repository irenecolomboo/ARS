import math
import pygame

class Sensors:
    def __init__(self, robot, max_distance=200):
        self.robot = robot
        self.num_sensors = 12
        self.max_distance = max_distance
        self.angles = [math.radians(i * 30) for i in range(self.num_sensors)]
        self.readings = [self.max_distance for _ in range(self.num_sensors)]

    def update(self, walls):
        self.readings = []

        for angle in self.angles:
            # Sensor direction
            sensor_angle = self.robot.angle + angle
            start = (self.robot.position[0], self.robot.position[1])
            end = (
                start[0] + self.max_distance * math.cos(sensor_angle),
                start[1] - self.max_distance * math.sin(sensor_angle)
            )

            min_distance = self.max_distance

            # Check intersection with all walls
            for wall in walls:
                intersect_point = self.get_line_intersection(start, end, (wall[0], wall[1]), (wall[2], wall[3]))
                if intersect_point:
                    distance = math.hypot(intersect_point[0] - start[0], intersect_point[1] - start[1])
                    if distance < min_distance:
                        min_distance = distance

            print("Sensor readings:", [round(r, 1) for r in self.readings])

            self.readings.append(min_distance)

    def get_line_intersection(self, p1, p2, q1, q2):
        # Line-line intersection algorithm
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = q1
        x4, y4 = q2

        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denom == 0:
            return None  # Parallel lines

        px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / denom
        py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / denom

        # Check if intersection point is within both segments
        if (min(x1, x2) <= px <= max(x1, x2) and
            min(y1, y2) <= py <= max(y1, y2) and
            min(x3, x4) <= px <= max(x3, x4) and
            min(y3, y4) <= py <= max(y3, y4)):
            return (px, py)
        return None

    def draw(self, screen):
        # Font for sensor values
        font = pygame.font.SysFont(None, 16)

        for i, angle in enumerate(self.angles):
            # Sensor direction
            sensor_angle = self.robot.angle + angle
            start_x, start_y = self.robot.position

            # Distance reading for this sensor
            distance = self.readings[i]

            # Endpoint of the sensor line based on the reading
            end_x = start_x + distance * math.cos(sensor_angle)
            end_y = start_y - distance * math.sin(sensor_angle)

            # Draw sensor line
            pygame.draw.line(screen, (100, 100, 255), (start_x, start_y), (end_x, end_y), 1)

            # Draw sensor reading text, placed at the end of the sensor line
            text_surface = font.render(str(int(distance)), True, (0, 0, 0))
            text_rect = text_surface.get_rect(center=(end_x, end_y))
            screen.blit(text_surface, text_rect)

