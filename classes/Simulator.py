import pygame

class Simulator:
    def __init__(self, width=800, height=600):
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Mobile Robot Simulator")
        self.clock = pygame.time.Clock()
        self.running = True

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

    def update_display(self):
        pygame.display.flip()

    def clear_screen(self, color=(255, 255, 255)):
        self.screen.fill(color)

    def tick(self, fps=60):
        self.clock.tick(fps)

    def quit(self):
        pygame.quit()
