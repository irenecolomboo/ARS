from classes.Simulator import Simulator
from classes.Robot import Robot
from classes.Environment import Environment
from classes.Collision import CollisionHandler
import pygame

def main():
    sim = Simulator()
    environment = Environment(sim.screen.get_width(), sim.screen.get_height())
    collision_handler = CollisionHandler(environment)
    robot = Robot(position=[400, 300], environment=environment, collision_handler=collision_handler, radius=20)

    while sim.running:
        sim.handle_events()
        keys = pygame.key.get_pressed()

        if keys[pygame.K_r]:
            robot.reset()

        robot.handle_keys()
        robot.update()

        sim.clear_screen()

        environment.draw(sim.screen)

        robot.draw(sim.screen)

        sim.update_display()
        sim.tick()

    sim.quit()

if __name__ == "__main__":
    main()
