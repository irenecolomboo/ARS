import numpy as np
import math

class OccupancyGridMap:
    def __init__(self, width_m, height_m, resolution):
        """
        Initialize the occupancy grid map.

        Args:
            width_m (float): Width of the map in meters.
            height_m (float): Height of the map in meters.
            resolution (float): Size of one grid cell in meters.
        """
        self.resolution = resolution
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
        """Convert log-odds to a probability value."""
        return 1 - 1 / (1 + np.exp(l))

    def update_from_scan(self, robot_pose, scan_points, max_range, l_occ=0.85, l_free=-0.4):
        """
        Update log-odds of each cell based on sensor data.

        Args:
            robot_pose: (x, y) position of the robot in meters
            scan_points: list of (x, y) points where features were detected
            max_range: maximum sensor range (circular scan radius)
            l_occ: log-odds to add for occupied cells
            l_free: log-odds to add for free cells
        """
        rx, ry = robot_pose
        ix_r, iy_r = self.world_to_map(rx, ry)

        for iy in range(self.height_cells):
            for ix in range(self.width_cells):
                x_cell, y_cell = self.map_to_world(ix, iy)

                dx = x_cell - rx
                dy = y_cell - ry
                distance = math.hypot(dx, dy)

                if distance <= max_range:
                    # Check if this cell is one of the scanned obstacle points
                    is_obstacle = False
                    for zx, zy in scan_points:
                        if abs(zx - x_cell) < self.resolution / 2 and abs(zy - y_cell) < self.resolution / 2:
                            is_obstacle = True
                            break

                    if is_obstacle:
                        self.grid[iy, ix] += l_occ - self.l0
                    else:
                        self.grid[iy, ix] += l_free - self.l0
