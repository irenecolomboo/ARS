import pygame
import numpy as np
from classes.Simulator import Simulator
from classes.Robot import Robot
from classes.Environment import Environment
from classes.Collision import CollisionHandler
from classes.GeneticAlg import run_genetic_algorithm, simulate_path


def main():
    pygame.init()

    frame_counter = 0
    update_interval = 30  # frames between recomputing GA

    sim = Simulator()
    env_width = sim.screen.get_width()
    env_height = sim.screen.get_height()

    environment = Environment(env_width, env_height)
    collision_handler = CollisionHandler(environment)
    robot = Robot(position=[50, 50], environment=environment, collision_handler=collision_handler, radius=20)

    scale = 4
    grid_surface_width = robot.occupancy_map.width_cells * scale
    grid_surface_height = robot.occupancy_map.height_cells * scale
    grid_surface = pygame.Surface((grid_surface_width, grid_surface_height))

    combined_width = env_width + grid_surface_width
    combined_height = max(env_height, grid_surface_height)
    combined_display = pygame.display.set_mode((combined_width, combined_height))
    pygame.display.set_caption("Robot + Occupancy Grid")

    best_path = None
    best_path_points = []
    goal = (88, 38)  # goal in grid coords
    goal_px = (
        goal[0] * robot.occupancy_map.resolution,
        goal[1] * robot.occupancy_map.resolution
        )

    # === Auto-run GA on startup ===
    grid = robot.occupancy_map.grid
    start_px = robot.position
    start = (int(start_px[0] // robot.occupancy_map.resolution),
             int(start_px[1] // robot.occupancy_map.resolution))


    best_path = run_genetic_algorithm(start, goal, grid)
    best_path_points = simulate_path(best_path, start)
    print("Auto-started best path.")

    path_step_index = 0
    follow_path = True
    target_px = None  # ← put it here ONCE, outside the loop


    while sim.running:
        sim.handle_events()
        keys = pygame.key.get_pressed()

        if keys[pygame.K_r]:
            robot.reset()

        if not follow_path:
            robot.handle_keys()

        robot.update()

        # === Stop at goal ===
        dx = abs(robot.position[0] - goal_px[0])
        dy = abs(robot.position[1] - goal_px[1])
        if dx < 8 and dy < 8:
            follow_path = False
            robot.V_l = 0
            robot.V_r = 0
            print("Reached goal!")


        # === Follow path logic ===
        if follow_path and best_path is not None:
            if target_px is None and path_step_index < len(best_path):
                for move in best_path:
                    #move = best_path[path_step_index]
                    robot.follow_move_command(move)

                    robot.update()
                    robot.draw(sim.screen, grid_surface)

                    # Set target pixel position
                    target_px = robot.position.copy()
                    gx, gy = best_path_points[path_step_index]
                    target_px = [
                        gx * robot.occupancy_map.resolution,
                        gy * robot.occupancy_map.resolution
                    ]

                    print(f"[MOVE] Step : {move} → Target {target_px}")

            if target_px is not None:
                dx = abs(robot.position[0] - target_px[0])
                dy = abs(robot.position[1] - target_px[1])
                if dx < 2 and dy < 2:  # close enough to target
                    path_step_index += 1
                    target_px = None  # ready for next move

            if path_step_index >= len(best_path):
                follow_path = False
                robot.V_l = 0
                robot.V_r = 0
                print("Finished path.")

        # === Drawing ===
        sim.clear_screen()
        environment.draw(sim.screen)
        robot.draw(sim.screen, grid_surface)

        if best_path_points:
            for (gx, gy) in best_path_points:
                px = gx * robot.occupancy_map.resolution
                py = gy * robot.occupancy_map.resolution
                pygame.draw.circle(sim.screen, (0, 255, 0), (int(px), int(py)), 3)

        goal_px = (goal[0] * robot.occupancy_map.resolution, goal[1] * robot.occupancy_map.resolution)
        pygame.draw.circle(sim.screen, (255, 0, 0), (int(goal_px[0]), int(goal_px[1])), 5)
        if target_px:
            pygame.draw.circle(sim.screen, (0, 0, 255), (int(target_px[0]), int(target_px[1])), 4)


        grid_surface.fill((128, 128, 128))
        robot.occupancy_map.draw_map_on_screen(grid_surface, scale=scale)

        combined_display.blit(sim.screen, (0, 0))
        combined_display.blit(grid_surface, (env_width, 0))

        frame_counter += 1
        if frame_counter % update_interval == 0:
            grid = robot.occupancy_map.grid
            start_px = robot.position
            start = (int(start_px[0] // robot.occupancy_map.resolution),
                     int(start_px[1] // robot.occupancy_map.resolution))

            best_path = run_genetic_algorithm(start, goal, grid)
            print("Full path:", best_path)
            best_path_points = simulate_path(best_path, start)

            path_step_index = 0
            follow_path = True
            target_px = None
            print(f"Recomputed path at frame {frame_counter}")

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