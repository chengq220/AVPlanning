import pygame 


class environemnt():
    """
    A class that simulates the environment for the autonomous vehicle 
    """
    def __init__(self, dim, ourself = None):
        """
        Initialize the environment

        Args:
            dim (tuple): The dimension of the screen (width, height)
            ourself (tuple): The tuple of parameters describing the vehicle we are interested in 
        """
        self.screen = self.initDisplay(dim[0], dim[1])

    def initDisplay(self, width, height):
        pygame.init()
        screen = pygame.display.set_mode((width, height))
        return screen

    def runGame(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            pygame.display.update()
        pygame.quit()
