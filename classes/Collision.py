import math

class CollisionHandler:
    def __init__(self, environment):
        self.environment = environment

    def handle_collision(self, robot, proposed_position):
        full_motion = (
            proposed_position[0] - robot.position[0],
            proposed_position[1] - robot.position[1]
        )

        motion_length = math.hypot(*full_motion)
        if motion_length == 0:
            return robot.position

        step_size = robot.radius / 3
        num_steps = max(1, int(motion_length / step_size))
        step_vector = (full_motion[0] / num_steps, full_motion[1] / num_steps)
        current_position = robot.position

        for _ in range(num_steps):
            next_position = (
                current_position[0] + step_vector[0],
                current_position[1] + step_vector[1]
            )

            collisions = []
            for wall in self.environment.get_walls():
                if self.will_collide(robot,
                                     (next_position[0] - current_position[0], next_position[1] - current_position[1]),
                                     wall):
                    collisions.append(wall)

            if collisions:
                precise_contact_point = current_position
                for wall in collisions:
                    precise_contact_point = self.find_precise_contact(robot, current_position, next_position, wall)

                # Try moving along each wall direction to escape collision
                for wall in collisions:
                    wall_vector = (wall[2] - wall[0], wall[3] - wall[1])
                    wall_length = math.hypot(*wall_vector)
                    if wall_length == 0:
                        continue

                    wall_unit = (wall_vector[0] / wall_length, wall_vector[1] / wall_length)

                    remaining_motion = (
                        next_position[0] - precise_contact_point[0],
                        next_position[1] - precise_contact_point[1]
                    )
                    dot_product = remaining_motion[0] * wall_unit[0] + remaining_motion[1] * wall_unit[1]
                    projected_motion = (dot_product * wall_unit[0], dot_product * wall_unit[1])

                    trial_position = (
                        precise_contact_point[0] + projected_motion[0],
                        precise_contact_point[1] + projected_motion[1]
                    )

                    # Check if it gets trapped
                    if not any(self.will_collide(robot, (
                    trial_position[0] - current_position[0], trial_position[1] - current_position[1]), w) for w in
                               self.environment.get_walls()):
                        current_position = trial_position
                        break
                # Fallback Behavior
                else:
                    # Stop completely if no escape route found
                    return precise_contact_point

            else:
                current_position = next_position

        return current_position

    def will_collide(self, robot, motion_vector, wall):
        next_pos = (
            robot.position[0] + motion_vector[0],
            robot.position[1] + motion_vector[1]
        )

        # Check distance from next position to wall
        distance = self.distance_to_wall(next_pos, wall)
        return distance <= robot.radius


    def distance_to_wall(self, circle_pos, wall):
        # return the distance!
        x1, y1, x2, y2 = wall
        cx, cy = circle_pos

        line_len = math.hypot(x2 - x1, y2 - y1)
        if line_len == 0:
            return float('inf')

        t = max(0, min(1, ((cx - x1) * (x2 - x1) + (cy - y1) * (y2 - y1)) / (line_len ** 2)))
        nearest_x = x1 + t * (x2 - x1)
        nearest_y = y1 + t * (y2 - y1)

        return math.hypot(nearest_x - cx, nearest_y - cy)


    def find_precise_contact(self, robot, safe_pos, colliding_pos, wall, iterations=10):
        # Binary search between safe_pos and colliding_pos
        for _ in range(iterations):
            mid_pos = (
                (safe_pos[0] + colliding_pos[0]) / 2,
                (safe_pos[1] + colliding_pos[1]) / 2
            )
            distance = self.distance_to_wall(mid_pos, wall)

            if distance < robot.radius:
                # Still inside wall: go back
                colliding_pos = mid_pos
            else:
                # Safe: move forward
                safe_pos = mid_pos

        return safe_pos