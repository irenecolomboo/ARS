import numpy as np
import math
import pygame

class OccupancyGridMap:
    def __init__(self, width_m, height_m, resolution):
        """
        Initialize the occupancy grid map.

        Args:
            width_m (float): Width of the map in meters.
            height_m (float): Height of the map in meters.
            resolution (float): Size of one grid cell in meters.
        """
        self.resolution = 8
        self.width_cells = int(np.ceil(width_m / resolution))
        self.height_cells = int(np.ceil(height_m / resolution))
        self.l0 = self.prob_to_log_odds(0.5)  # log odds of 0.5 = 0
        self.grid = np.full((self.height_cells, self.width_cells), self.l0, dtype=np.float32)

    def world_to_map(self, x, y):
        """Convert world coordinates (meters) to grid indices (ix, iy)."""
        ix = int(x / self.resolution)
        iy = int(y / self.resolution)
        return ix, iy

    def map_to_world(self, ix, iy):
        """Convert grid indices (ix, iy) to world coordinates (meters)."""
        x = ix * self.resolution
        y = iy * self.resolution
        return x, y

    def prob_to_log_odds(self, p):
        """Convert a probability to log-odds representation."""
        return np.log(p / (1 - p))
    
    def log_odds_to_prob(self, l):
        l = np.clip(l, -100, 100)  # prevent overflow in exp
        return 1 - 1 / (1 + np.exp(l))

    
    def update_from_scan(self, robot_pose, scan_points, max_range, l_occ=0.85, l_free=-0.4):
        rx, ry = robot_pose
        ix_r, iy_r = self.world_to_map(rx, ry)

        for iy in range(self.height_cells):
            for ix in range(self.width_cells):
                x_cell, y_cell = self.map_to_world(ix, iy)
                dx = x_cell - rx
                dy = y_cell - ry
                distance = math.hypot(dx, dy)

                if distance <= max_range:
                    is_obstacle = False
                    for zx, zy in scan_points:
                        if abs(zx - x_cell) < self.resolution / 2 and abs(zy - y_cell) < self.resolution / 2:
                            is_obstacle = True
                            break

                    delta = l_occ - self.l0 if is_obstacle else l_free - self.l0
                    self.grid[iy, ix] += delta

                    # CLAMP: prevent log-odds from blowing up
                    self.grid[iy, ix] = np.clip(self.grid[iy, ix], -10, 10)

    def draw_map_on_screen(self, screen, scale=4):  
        prob_grid = self.get_probability_grid()
        height, width = prob_grid.shape

        for y in range(height):
            for x in range(width):
                p = prob_grid[y, x]
                if p == 0.5:
                    color = (100, 100, 100)  # unexplored = gray
                elif p > 0.7:
                    color = (0, 0, 0)  # occupied = black
                elif p < 0.3:
                    color = (255, 255, 255)  # free = white
                else:
                    value = int((1 - p) * 255)
                    color = (value, value, value)  # in-between

                rect = pygame.Rect(x * scale, y * scale, scale, scale)
                pygame.draw.rect(screen, color, rect)


    def get_probability_grid(self):
        """
        Return a grid of probabilities from the log-odds grid.
        """
        return self.log_odds_to_prob(self.grid)

