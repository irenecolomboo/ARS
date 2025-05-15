import heapq
import numpy as np

def heuristic(a, b):
    # Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal, threshold=0.6, resolution=1.0):
    """
    grid: 2D numpy array of probabilities (0=free, 1=occupied)
    start, goal: (x, y) in world coordinates
    threshold: above this prob is considered occupied
    resolution: meters per grid cell (from OccupancyGridMap)
    """
    def world_to_map(x, y):
        return int(x / resolution), int(y / resolution)

    def map_to_world(ix, iy):
        return ix * resolution, iy * resolution

    # Convert world to grid
    start_map = world_to_map(*start)
    goal_map = world_to_map(*goal)

    neighbors = [(-1,0),(1,0),(0,-1),(0,1)]  # 4-connected
    open_set = []
    heapq.heappush(open_set, (heuristic(start_map, goal_map), 0, start_map, None))
    
    came_from = {}
    cost_so_far = {start_map: 0}

    while open_set:
        _, cost, current, parent = heapq.heappop(open_set)

        if current == goal_map:
            # Reconstruct path
            path = [map_to_world(*current)]
            while parent:
                path.append(map_to_world(*parent))
                parent = came_from[parent]
            return path[::-1]  # in world coordinates

        if current in came_from:
            continue
        came_from[current] = parent

        for dx, dy in neighbors:
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                if grid[ny, nx] > threshold:
                    continue  # blocked
                new_cost = cost + 1
                if (nx, ny) not in cost_so_far or new_cost < cost_so_far[(nx, ny)]:
                    cost_so_far[(nx, ny)] = new_cost
                    priority = new_cost + heuristic((nx, ny), goal_map)
                    heapq.heappush(open_set, (priority, new_cost, (nx, ny), current))

    return None  # no path found
