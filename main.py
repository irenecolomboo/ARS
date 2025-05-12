import pygame
from classes.Simulator import Simulator
from classes.Robot import Robot
from classes.Environment import Environment
from classes.Collision import CollisionHandler

def main():
    pygame.init()

    # Original simulator screen (for environment and robot)
    sim = Simulator()
    env_width = sim.screen.get_width()
    env_height = sim.screen.get_height()

    # Create environment and robot
    environment = Environment(env_width, env_height)
    collision_handler = CollisionHandler(environment)
    robot = Robot(position=[50, 50], environment=environment, collision_handler=collision_handler, radius=20)

    # Create a separate surface for occupancy grid
    scale = 4  # how much to scale the grid map
    grid_surface_width = robot.occupancy_map.width_cells * scale
    grid_surface_height = robot.occupancy_map.height_cells * scale
    grid_surface = pygame.Surface((grid_surface_width, grid_surface_height))

    # Create a combined window to hold both views
    combined_width = env_width + grid_surface_width
    combined_height = max(env_height, grid_surface_height)
    combined_display = pygame.display.set_mode((combined_width, combined_height))
    pygame.display.set_caption("Robot + Occupancy Grid")

    while sim.running:
        sim.handle_events()
        keys = pygame.key.get_pressed()

        if keys[pygame.K_r]:
            robot.reset()

        robot.handle_keys()
        robot.update()

        # ---- Draw environment to left surface ----
        sim.clear_screen()
        environment.draw(sim.screen)
        robot.draw(sim.screen, grid_surface)

        # ---- Draw occupancy grid to right surface ----
        grid_surface.fill((128, 128, 128))  # gray for unexplored
        robot.occupancy_map.draw_map_on_screen(grid_surface, scale=scale)

        # ---- Combine both surfaces onto the display ----
        combined_display.blit(sim.screen, (0, 0))
        combined_display.blit(grid_surface, (env_width, 0))

        pygame.display.flip()
        sim.tick()

    sim.quit()

if __name__ == "__main__":
    main()
