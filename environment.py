import pygame 
import numpy as np
import random
from solver import AVP

PLAYER = (0, 0, 255) #blue color
OBSTACLE = (0, 255, 0) #green color
ENV = (255, 0, 0) #red color
SCREEN = (0, 0, 0) #black color

class environemnt():
    """
    A class that simulates the environment for the autonomous vehicle 
    """
    def __init__(self, dim, timeStep = 1, numVehicles = 1, maxVel = 1):
        """
        Initialize the environment

        Args:
            dim (int): The dimension of the screen 
            timeStep (int): The time step for each iteration
            numVehicles (int): The number of vehicles on the road that we have to avoid
            maxVel (int): The maximum velocity that the vehicle can achieve
        """
        self.dim = dim
        self.step = timeStep
        self.maxVel = maxVel
        self.screen = self.initDisplay(dim[0], dim[1])
        self.road = self.__generateRoad()
        self.state, self.control = self.__generateVehicle(numVehicles)
        self.optimizer = AVP() 

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
            Dictionaries containing information about each vehicle generated
        """
        state = dict()
        control = dict()
        for i in range(numVehicle+1):
            #generate the player vehicle:
            if i == 0:
                radius = 20
            else: #Generate the obstacle vehicles
                radius = 5 * random.randint(3,8)
            maxSize = 50
            # velocity, acceleration, angle, angular velocity
            v = random.uniform(0.5,0.8) * random.uniform(0,1)
            a = 0
            theta = 0
            w = 0
            
            # x,y position
            alpha = random.randint(0,1)
            x = random.randint(0, self.dim[1])
            y = alpha * random.randint(self.road[0] + maxSize, self.road[0] + self.road[1] -maxSize)\
                + (1-alpha) * random.randint(self.road[0]-self.road[1]+maxSize, self.road[0] - maxSize)
    
            curState = (x, y, theta, v, radius)
            curControl = (w, a)
            state[i] = curState
            control[i] = curControl
            pygame.draw.circle(self.screen, PLAYER if i == 0 else OBSTACLE, (x,y), radius)
        return state, control 
    
    def __update(self):
        """
        Update the new position of the vehicles
        """
        self.control[0] = self.__optimize()
        for key in self.state.keys():
            # update for the parameters according to formulas for velocity and angular velocity
            x0,y0,theta0,v0,radius = self.state[key] 
            a, w = self.control[key]
            v = v0 + a * self.step
            theta = theta0 + w * self.step
            x = (x0 + v * np.cos(theta))
            y = (y0 + v * np.sin(theta))
            if(key != 0): #only make the obstacle circle back
                x = x % self.dim[0]
                y = y % self.dim[1] 
            self.state[key] = (x,y, theta, v, radius)

    def __optimize(self):
        """
        Solves the nonlinear optimization problem to find the best 
        control vector for a given time stamp
        """
        guess = np.array([-1.2, 1.0])
        solver = self.optimizer
        sol, _ = solver.solve(guess)
        self.control[0] = sol

    def __refreshFrame(self):
        """
        Refreshes the screen with updated values
        """
        self.screen.fill(SCREEN) 
        self.__generateRoad()
        self.__update()
        for key in self.state.keys():
            x,y,_,_, radius = self.state[key]
            pygame.draw.circle(self.screen, PLAYER if key == 0 else OBSTACLE, (x,y), radius)
        pygame.display.update()

    def runGame(self):
        """
        Run the simulation/game
        """
        running = True
        while running and self.state[0][0] <= self.dim[0]:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            self.__refreshFrame()
        pygame.quit()
