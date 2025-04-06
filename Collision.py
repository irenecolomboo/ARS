import math

class CollisionHandler:
    def __init__(self, environment):
        self.environment = environment

    def handle_collision(self, robot, proposed_position):
        collisions = []

        for wall in self.environment.get_walls():
            if self.circle_line_collision(proposed_position, robot.radius, wall):
                collisions.append(wall)

        if not collisions:
            # No collision, accept motion
            return proposed_position

        if len(collisions) > 1:
            # Colliding with multiple walls: stop movement
            return robot.position

        # Single collision: slide along wall
        wall = collisions[0]

        # Compute wall vector
        wall_vector = (wall[2] - wall[0], wall[3] - wall[1])
        wall_length = math.hypot(*wall_vector)
        if wall_length == 0:
            return robot.position  # invalid wall

        wall_unit = (wall_vector[0] / wall_length, wall_vector[1] / wall_length)

        # Project motion onto wall direction
        motion_vector = (
            proposed_position[0] - robot.position[0],
            proposed_position[1] - robot.position[1]
        )
        dot_product = motion_vector[0] * wall_unit[0] + motion_vector[1] * wall_unit[1]
        corrected_motion = (dot_product * wall_unit[0], dot_product * wall_unit[1])

        # Apply corrected motion
        return (
            robot.position[0] + corrected_motion[0],
            robot.position[1] + corrected_motion[1]
        )


    def circle_line_collision(self, circle_pos, circle_radius, wall):
        # Simplified algorithm: check distance from wall to circle center
        x1, y1, x2, y2 = wall
        cx, cy = circle_pos

        # Compute line segment length
        line_len = math.hypot(x2 - x1, y2 - y1)
        if line_len == 0:
            return False  # Not a valid wall

        # Project circle center onto wall segment
        t = max(0, min(1, ((cx - x1) * (x2 - x1) + (cy - y1) * (y2 - y1)) / (line_len ** 2)))
        nearest_x = x1 + t * (x2 - x1)
        nearest_y = y1 + t * (y2 - y1)

        # Check distance from circle center to nearest point
        dist = math.hypot(nearest_x - cx, nearest_y - cy)

        return dist <= circle_radius
