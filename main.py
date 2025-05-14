import pygame
import numpy as np
from classes.Simulator import Simulator
from classes.Robot import Robot
from classes.Environment import Environment
from classes.Collision import CollisionHandler
from classes.GeneticAlg import run_genetic_algorithm, simulate_path


def main():
    pygame.init()

    sim = Simulator()
    env_width = sim.screen.get_width()
    env_height = sim.screen.get_height()

    environment = Environment(env_width, env_height, wall_thickness=10)
    collision_handler = CollisionHandler(environment)

    # === MULTI-AGENT SETUP ===
    robot_starts = [[50, 50], [700, 200], [100, 550]]
    goals = [(88, 38), (15, 10), (30, 30)]
    robots = []
    paths = []
    path_points = []
    step_indices = []
    targets = []
    follow_flags = []

    frame_counter = 0
    update_interval = 30  # adjust as needed

    for start_pos, goal in zip(robot_starts, goals):
        robot = Robot(position=start_pos, environment=environment, collision_handler=collision_handler, radius=20)
        grid = robot.occupancy_map.grid
        start_grid = (int(start_pos[0] // robot.occupancy_map.resolution),
                      int(start_pos[1] // robot.occupancy_map.resolution))
        path = run_genetic_algorithm(start_grid, goal, grid)
        point_path = simulate_path(path, start_grid)

        robots.append(robot)
        paths.append(path)
        path_points.append(point_path)
        step_indices.append(0)
        targets.append(None)
        follow_flags.append(True)

    # === Display setup ===
    scale = 4
    grid_surface_width = robots[0].occupancy_map.width_cells * scale
    grid_surface_height = robots[0].occupancy_map.height_cells * scale
    grid_surface = pygame.Surface((grid_surface_width, grid_surface_height))

    combined_width = env_width + grid_surface_width
    combined_height = max(env_height, grid_surface_height)
    combined_display = pygame.display.set_mode((combined_width, combined_height))
    pygame.display.set_caption("Multi-Agent Robots + Occupancy Grid")

    # === MAIN LOOP ===
    running = True
    while running:
        sim.handle_events()
        keys = pygame.key.get_pressed()
        if keys[pygame.K_r]:
            for r in robots:
                r.reset()

        sim.clear_screen()
        environment.draw(sim.screen)

        grid_surface.fill((128, 128, 128))

        for i, robot in enumerate(robots):
            robot.update()

            if follow_flags[i] and paths[i]:
                if targets[i] is None and step_indices[i] < len(paths[i]):
                    move = paths[i][step_indices[i]]
                    robot.follow_move_command(move)
                    gx, gy = path_points[i][step_indices[i]]
                    targets[i] = [gx * robot.occupancy_map.resolution,
                                  gy * robot.occupancy_map.resolution]
                    print(f"[Robot {i}] Step {step_indices[i]}: {move} → Target {targets[i]}")

                if targets[i] is not None:
                    dx = abs(robot.position[0] - targets[i][0])
                    dy = abs(robot.position[1] - targets[i][1])
                    if dx < 2 and dy < 2:
                        step_indices[i] += 1
                        targets[i] = None

                if step_indices[i] >= len(paths[i]):
                    follow_flags[i] = False
                    robot.V_l = 0
                    robot.V_r = 0
                    print(f"[Robot {i}] Finished path.")

            # Draw the goal point
            goal = goals[i]
            goal_px = (
                goal[0] * robot.occupancy_map.resolution,
                goal[1] * robot.occupancy_map.resolution
            )
            pygame.draw.circle(sim.screen, (255, 0, 0), (int(goal_px[0]), int(goal_px[1])), 5)


            robot.draw(sim.screen, grid_surface)

            # Path + target drawing
            for (gx, gy) in path_points[i]:
                px = gx * robot.occupancy_map.resolution
                py = gy * robot.occupancy_map.resolution
                pygame.draw.circle(sim.screen, (0, 255, 0), (int(px), int(py)), 2)

            if targets[i]:
                pygame.draw.circle(sim.screen, (0, 0, 255), (int(targets[i][0]), int(targets[i][1])), 4)

        robots[0].occupancy_map.draw_map_on_screen(grid_surface, scale=scale)

        combined_display.blit(sim.screen, (0, 0))
        combined_display.blit(grid_surface, (env_width, 0))


        frame_counter += 1
        if frame_counter % update_interval == 0:
            for i, robot in enumerate(robots):
                start_px = robot.position
                start = (int(start_px[0] // robot.occupancy_map.resolution),
                        int(start_px[1] // robot.occupancy_map.resolution))
                goal = goals[i]

                grid = robot.occupancy_map.grid
                path = run_genetic_algorithm(start, goal, grid)
                point_path = simulate_path(path, start)

                paths[i] = path
                path_points[i] = point_path
                step_indices[i] = 0
                targets[i] = None
                follow_flags[i] = True
                print(f"[Robot {i}] Recomputed path at frame {frame_counter}")


        pygame.display.flip()
        sim.tick()

    sim.quit()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
        input("Press Enter to exit...")
