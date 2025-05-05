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
    
    def bresenham(self, x0, y0, x1, y1):
        """Yield grid cells along the line from (x0, y0) to (x1, y1) using Bresenham's algorithm."""
        x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            yield x0, y0
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy


    
    def update_from_scan(self, robot_pos, scan_points, max_range, l_occ=0.85, l_free=-0.4):
        rx, ry = robot_pos
        ix_r, iy_r = self.world_to_map(rx, ry)

        for x_hit, y_hit in scan_points:
            ix_hit, iy_hit = self.world_to_map(x_hit, y_hit)

            # Ray trace from robot to hit point using Bresenham
            for ix, iy in self.bresenham(ix_r, iy_r, ix_hit, iy_hit):
                if 0 <= iy < self.height_cells and 0 <= ix < self.width_cells:
                    self.grid[iy, ix] += l_free  # mark as free

            # Mark the final cell as occupied
            if 0 <= iy_hit < self.height_cells and 0 <= ix_hit < self.width_cells:
                self.grid[iy_hit, ix_hit] += l_occ

        # Clamp grid values to avoid overflow
        self.grid = np.clip(self.grid, -10, 10)


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

