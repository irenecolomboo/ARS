import math
import pygame
from shapely.geometry import LineString

class Sensors:
    def __init__(self, robot, max_distance=200):
        self.robot = robot
        self.num_sensors = 12
        self.max_distance = max_distance
        self.angles = [math.radians(i * 30) for i in range(self.num_sensors)]
        self.readings = [self.max_distance for _ in range(self.num_sensors)]

    def update(self, walls):
        new_readings = []

        for angle in self.angles:
            # Sensor direction
            sensor_angle = self.robot.angle + angle
            start = self.robot.position
            end = (
                start[0] + self.max_distance * math.cos(sensor_angle),
                start[1] - self.max_distance * math.sin(sensor_angle)
            )

            min_distance = self.max_distance
            closest_point = None

            # Check intersection with all walls
            for wall in walls:
                intersect_point = self.get_line_intersection(start, end, (wall[0], wall[1]), (wall[2], wall[3]))
                if intersect_point:
                    distance = math.hypot(intersect_point[0] - start[0], intersect_point[1] - start[1])
                    if distance < min_distance:
                        min_distance = distance
                        closest_point = intersect_point

            new_readings.append(min_distance)

        self.readings = new_readings

        print("Sensor readings:", [round(r, 1) for r in self.readings])

    def get_line_intersection(self, p1, p2, q1, q2):
        line1 = LineString([p1, p2])
        line2 = LineString([q1, q2])
        intersection = line1.intersection(line2)

        if intersection.is_empty:
            return None
        if intersection.geom_type == 'Point':
            return (intersection.x, intersection.y)

        # If it's a LineString (overlapping), return closest point to sensor start
        if intersection.geom_type == 'LineString':
            points = list(intersection.coords)
            points.sort(key=lambda point: math.hypot(point[0] - p1[0], point[1] - p1[1]))
            return points[0]

        return None


    def draw(self, screen):
        font = pygame.font.SysFont(None, 16)

        for i, angle in enumerate(self.angles):
            sensor_angle = self.robot.angle + angle
            start_x, start_y = self.robot.position

            # Clip distance at robot radius (if robot is against the wall, sensor reads zero)
            distance = self.readings[i]
            distance = max(0, distance - self.robot.radius)

            end_x = start_x + distance * math.cos(sensor_angle)
            end_y = start_y - distance * math.sin(sensor_angle)

            pygame.draw.line(screen, (100, 100, 255), (start_x, start_y), (end_x, end_y), 1)

            text_surface = font.render(str(int(distance)), True, (0, 0, 0))
            text_rect = text_surface.get_rect(center=(end_x, end_y))
            screen.blit(text_surface, text_rect)

