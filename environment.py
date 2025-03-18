import pygame 
import numpy as np
import random
from solver import AVP, visualizeSolutionPosition

PLAYER = (0, 0, 255) #blue color
OBSTACLE = (0, 255, 0) #green color
ENV = (255, 0, 0) #red color
SCREEN = (0, 0, 0) #black color

class environemnt():
    """
    A class that simulates the environment for the autonomous vehicle 
    """
    def __init__(self, dim, numStep = 50, numVehicles = 1, maxVel = 1, radius = 25):
        """
        Initialize the environment

        Args:
            dim (int): The dimension of the screen 
            timeStep (int): The time step for each iteration
            numVehicles (int): The number of vehicles on the road that we have to avoid
            maxVel (int): The maximum velocity that the vehicle can achieve
        """
        self.dim = dim
        self.numStep = numStep
        self.maxVel = maxVel
        self.radius = radius
        self.dim = dim
        self.numVehicle = numVehicles
        self.road = self.__calcRoad()
        self.vehicle = self.__generateVehicle()
        self.obsState = self.__generateObstacle(numVehicles)

    def __initDisplay(self, width, height):
        """
        Initialize the game screen

        Args:
            width (int): The width of the screen
            height (int): The height of the screen
        """
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
    
    def __calcRoad(self):
        """
        Return information about the boundar and the center

        Returns:
            A tuple containing the center and the boundary of the lane
        """
        roadCenter = self.dim[1] // 2
        roadWidth = self.dim[1] // 5
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

    def __generateVehicle(self):
        """
        Generate a vehicle that we are optimizing for

        Returns:
            Initial and end conditions 
        """
        maxSize = 50
        v = random.uniform(self.maxVel-5,self.maxVel-1) * random.uniform(0.8,1)
        a = 0
        theta = 0
        w = 0
        
        # x,y position
        alpha = random.randint(0,1)
        x = random.randint(0, self.dim[1]//8)
        y = alpha * random.randint(self.road[0] + maxSize, self.road[0] + self.road[1] -maxSize)\
            + (1-alpha) * random.randint(self.road[0]-self.road[1]+maxSize, self.road[0] - maxSize)
        initial = np.array([x, y, v, theta, w, a])
        end = np.array([self.road[0], y, v, theta, w, a])
        return (initial, end)

    def __generateObstacle(self, numVehicle):
        """
        Generate the obstacle vehicles on the two side of the road

        Args:
            numVehicle (int): The number of vehicle to generate

        Returns:
            Dictionaries containing information about each vehicle generated
        """
        obstacles = dict()
        for i in range(numVehicle):
            maxSize = 50
            # velocity, acceleration, angle, angular velocity
            v = random.uniform(self.maxVel-10,self.maxVel-5) 
            a = 0
            theta = 0
            w = 0
            # x,y position
            alpha = random.randint(0,1)
            x = random.randint(self.dim[1]//8 + 2*self.radius, self.dim[1] - self.dim[1]//4)
            y = alpha * random.randint(self.road[0] + maxSize, self.road[0] + self.road[1] -maxSize)\
                + (1-alpha) * random.randint(self.road[0]-self.road[1]+maxSize, self.road[0] - maxSize)
            feature = np.array([x, y, v, theta, w, a])
            obstacles[i] = feature
        return obstacles

    def optimize(self):
        """
        Solves the nonlinear optimization problem to find the best control vectors for the trajectory
        """
        radius = [self.radius] * (len(self.obsState.keys()) + 1)
        bounds = [(0, self.dim[1]), (self.dim[0]//2 - self.dim[0]//5, self.dim[0]//2 + self.dim[0]//5), \
                  (0, self.maxVel),(-np.pi/2, np.pi/2),(None, None),(None, None)]
        model = AVP(self.vehicle, self.obsState, radius, bounds, numStep=self.numStep,maxIter=10)
        sol = model.forward()
        self.sol = (sol[0::6], sol[1::6])
        self.obstacleTrajectory = model.obstacles
        # visualizeSolutionPosition(sol, model.obstacles)

    def __refreshFrame(self, timestep):
        """
        Refreshes the screen with updated values
        """
        self.screen.fill(SCREEN) 

        # Update the desire vehicle
        vx, vy = self.sol
        pygame.draw.circle(self.screen, PLAYER, (vx[timestep],vy[timestep]), self.radius)
        
        # Draw the road for each frame
        self.__drawDashedLine(ENV, stepsize=10)
        roadCenter, roadWidth = self.road
        pygame.draw.line(self.screen, ENV, (0, (roadCenter-roadWidth)), (self.dim[0], (roadCenter-roadWidth)), 1)
        pygame.draw.line(self.screen, ENV, (0, (roadCenter+roadWidth)), (self.dim[0], (roadCenter+roadWidth)), 1)

        # Update the obstacles 
        for idx in range(self.obstacleTrajectory.shape[0]):
            obs_x = self.obstacleTrajectory[idx, 6*timestep]
            obs_y = self.obstacleTrajectory[idx, 6*timestep + 1]
            pygame.draw.circle(self.screen, OBSTACLE, (obs_x,obs_y), self.radius)
        
        pygame.display.update()

    def runGame(self):
        """
        Run the simulation/game
        """
        running = True
        timestep = 0
        self.__initDisplay(self.dim[0], self.dim[1])
        while running and timestep < self.numStep:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            self.__refreshFrame(timestep)
            timestep = timestep + 1
            pygame.time.delay(75)
        pygame.quit()
