import pygame

class Environment:
    def __init__(self, width, height, wall_thickness=10):
        self.width = width
        self.height = height
        self.wall_thickness = wall_thickness

        # Borders
        self.walls = [
            (wall_thickness, wall_thickness, width - wall_thickness, wall_thickness),
            (width - wall_thickness, wall_thickness, width - wall_thickness, height - wall_thickness), 
            (width - wall_thickness, height - wall_thickness, wall_thickness, height - wall_thickness),  
            (wall_thickness, height - wall_thickness, wall_thickness, wall_thickness)  
        ]
        # 1 Empty plane
        # self.walls += [
        # ]


        # 2 Concave "C" shape in the middle left to check for concavity
        # self.walls += [
        #
        #     (200, 150, 200, 450),
        #     (200, 150, 400, 150),
        #     (200, 450, 400, 450),
        # ]

        # 3 maze
        self.walls += [

            (10, 100, 600, 100),
            (200, 200, 790, 200),
            (10, 300, 600, 300),
            (200, 400, 790, 400),
            (10, 500, 600, 500),
        ]


    def draw(self, screen):
        wall_color = (200, 0, 0)
        for wall in self.walls:
            pygame.draw.line(screen, wall_color, (wall[0], wall[1]), (wall[2], wall[3]), self.wall_thickness)

    def get_walls(self):
        return self.walls
