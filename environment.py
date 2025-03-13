import pygame 
import numpy as np
import random


PLAYER = (0, 0, 255) #blue color
OBSTACLE = (0, 255, 0) #green color
ENV = (255, 0, 0) #red color


class environemnt():
    """
    A class that simulates the environment for the autonomous vehicle 
    """
    def __init__(self, dim, ourself = None, numVehicles = 1):
        """
        Initialize the environment

        Args:
            dim (tuple): The dimension of the screen (width, height)
            ourself (tuple): The tuple of parameters describing the vehicle we are interested in 
            numVehicles (int): The number of vehicles on the road that we have to avoid
        """
        self.dim = dim
        self.screen = self.initDisplay(dim[0], dim[1])
        self.road = self.__generateRoad()
        self.vehicles = self.__generateVehicle(numVehicles)

    def initDisplay(self, width, height):
        """
        Initialize the game screen

        Args:
            width (int): The width of the screen
            height (int): The height of the screen

        Returns:
            The screen object
        """
        pygame.init()
        screen = pygame.display.set_mode((width, height))
        return screen
    
    def __generateRoad(self):
        """
        Generate the road and return information about the boundar and the center

        Returns:
            A tuple containing the center and the boundary of the lane
        """
        roadCenter = self.dim[1] // 2
        roadWidth = self.dim[1] // 5
        
        # Setup the road
        self.__drawDashedLine(ENV, stepsize=10)
        pygame.draw.line(self.screen, ENV, (0, (roadCenter-roadWidth)), (self.dim[0], (roadCenter-roadWidth)), 1)
        pygame.draw.line(self.screen, ENV, (0, (roadCenter+roadWidth)), (self.dim[0], (roadCenter+roadWidth)), 1)
        return (roadCenter, roadWidth)

    def __drawDashedLine(self, color, stepsize = 20):
        """
        Draw the dashed center line of the road

        Args:
            color (tuple): RGB value of the line
            stepSize (int): the size of the space in the dashed line 
        """
        yVal = np.arange(0, self.dim[0], stepsize)
        for i in range(yVal.shape[0]):
            if i%2 == 0:
                pygame.draw.line(self.screen, color, (yVal[i],self.dim[1]//2), (yVal[i] + stepsize, self.dim[1]//2), 1)

    def __generateVehicle(self, numVehicle):
        """
        Generate the obstacle vehicles on the two side of the road

        Args:
            numVehicle (int): The number of vehicle to generate

        Returns:
            A dictionary containing information about each vehicle generated
        """
        info = dict()
        for i in range(numVehicle):
            radius = 5 * random.randint(5,8)
            maxSize = 50
            v = random.randint(1,5)
            a = 0
            theta = 0
            alpha = random.randint(0,1)
            x = random.randint(0, self.dim[1])
            y = alpha * random.randint(self.road[0] + maxSize, self.road[0] + self.road[1] -maxSize)\
                + (1-alpha) * random.randint(self.road[0]-self.road[1]+maxSize, self.road[0] - maxSize)
            curInfo = (x, y, theta, v, a)
            info[i] = curInfo
            pygame.draw.circle(self.screen, OBSTACLE, (x,y), radius)
        return info 
    
    def __update():
        return False

    def runGame(self):
        """
        Run the simulation/game
        """
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            pygame.display.update()
        pygame.quit()
