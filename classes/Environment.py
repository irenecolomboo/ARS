import pygame

class Environment:
    def __init__(self, width, height, wall_thickness=10):
        self.width = width
        self.height = height
        self.wall_thickness = wall_thickness
        self.big_wall_thickness = 55

        # Borders
        self.walls = [
            (wall_thickness, wall_thickness, width - wall_thickness, wall_thickness),
            (width - wall_thickness, wall_thickness, width - wall_thickness, height - wall_thickness),
            (width - wall_thickness, height - wall_thickness, wall_thickness, height - wall_thickness),
            (wall_thickness, height - wall_thickness, wall_thickness, wall_thickness)
        ]
        self.big_walls = []
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
        # self.walls += [
        #
        #     (10, 100, 600, 100),
        #     (200, 200, 790, 200),
        #     (10, 300, 600, 300),
        #     (200, 400, 790, 400),
        #     (10, 500, 600, 500),
        # ]

        # self.landmarks = [
        #     (200, 10),  # (x, y)
        #     (500, 10),
        #     (500, 100),
        #     (350, 100),
        #     (50, 100),
        #     (300, 200),
        #     (500, 200),
        #     (700, 200),
        #     (200, 500),
        #     (600, 500)
        # ]

        # 4 Airport
        self.walls += [
            (10, 100, 200, 100),
            (200, 100, 200, 150),
            (300, 150, 300, 100),
            (300, 100, 400, 100),
            (400, 10, 400, 100),
            (10, 150, 200, 150),
            (300, 150, 790, 150),

            # Top cube on the right
            (300, 220, 690, 220),
            (300, 270, 690, 270),
            (300, 220, 300, 270),
            (690, 220, 690, 270),

            # Middle cube on the right
            (300, 340, 790, 340),
            (300, 390, 790, 390),
            (300, 340, 300, 390),
            (790, 340, 790, 390),

            # Last cube on the right
            (300, 460, 690, 460),
            (320, 510, 690, 510),
            (300, 460, 320, 510),
            (690, 460, 690, 510),

            # Top cube on the left
            (10, 220, 200, 220),
            (10, 270, 200, 270),
            (10, 220, 10, 270),
            (200, 220, 200, 270),

            # Middle cube on the left
            (10, 340, 200, 340),
            (10, 390, 200, 390),
            (10, 340, 10, 390),
            (200, 340, 200, 390),

            # Last cube on the left
            (10, 460, 200, 460),
            (10, 510, 220, 510),
            (10, 460, 10, 510),
            (200, 460, 220, 510),
        ]

        self.big_walls += [
            (300, 245, 690, 245),
        ]


        self.landmarks = [
            (200, 10),  # (x, y)
            (375, 125),
            (150, 125),
            (152, 365),
            (375, 365),
        ]



    def draw(self, screen):
        # Create a temporary surface with per-pixel alpha
        wall_surface = pygame.Surface(screen.get_size(), pygame.SRCALPHA)

        # Define a semi-transparent color (RGBA)
        wall_color = (0, 200, 0, 50)  # 128 makes it semi-transparent

        # Draw walls on the temporary surface
        for wall in self.walls:
            pygame.draw.line(wall_surface, wall_color, (wall[0], wall[1]), (wall[2], wall[3]), self.wall_thickness)

        # for wall in self.big_walls:
        #     pygame.draw.line(wall_surface, (200, 0, 0, 128), (wall[0], wall[1]), (wall[2], wall[3]), self.big_wall_thickness)

        # Blit the temporary surface onto the main screen
        screen.blit(wall_surface, (0, 0))

        # Draw landmarks normally
        for x, y in self.landmarks:
            pygame.draw.circle(screen, (0, 0, 255), (int(x), int(y)), 6)

    def get_walls(self):
        return self.walls + self.big_walls

    def get_landmarks(self):
        return self.landmarks