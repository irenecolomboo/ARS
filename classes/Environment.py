import pygame

class Environment:
    def __init__(self, width, height, wall_thickness=10):
        self.width = width
        self.height = height
        self.wall_thickness = wall_thickness

        # Define walls as line segments (x1, y1, x2, y2)
        self.walls = [
            (wall_thickness, wall_thickness, width - wall_thickness, wall_thickness),  # Top
            (width - wall_thickness, wall_thickness, width - wall_thickness, height - wall_thickness),  # Right
            (width - wall_thickness, height - wall_thickness, wall_thickness, height - wall_thickness),  # Bottom
            (wall_thickness, height - wall_thickness, wall_thickness, wall_thickness)  # Left
        ]

    def draw(self, screen):
        wall_color = (200, 0, 0)
        for wall in self.walls:
            pygame.draw.line(screen, wall_color, (wall[0], wall[1]), (wall[2], wall[3]), self.wall_thickness)

    def get_walls(self):
        return self.walls
