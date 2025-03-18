import pygame 
import numpy as np
import random
from solver import AVP, visualizeSolutionPosition
import json
from moviepy.video.io.ImageSequenceClip import ImageSequenceClip

PLAYER = (0, 0, 255) #blue color
OBSTACLE = (0, 255, 0) #green color
ENV = (255, 0, 0) #red color
SCREEN = (0, 0, 0) #black color

class environemnt():
    """
    A class that simulates the environment for the autonomous vehicle 
    """
    def __init__(self, env_file):
        """
        Initialize the environment
        """
        with open(env_file, "r") as json_file:
            data = json.load(json_file)
        self.dim = data["screenSize"]
        self.numStep = data["num_Step"]
        self.maxVel = data["max_velocity"]
        self.radius = data["radius"]
        self.numVehicle = data["num_obstacles"]
        self.road = data["road"]
        self.vehicle = (np.array(data["vehicle_start"]), np.array(data["vehicle_end"]))
        obs = dict()
        for idx, feature in enumerate(data["obstacles"]):
            obs[idx] = np.array(feature)
        self.obsState = obs        

    def __initDisplay(self, width, height):
        """
        Initialize the game screen

        Args:
            width (int): The width of the screen
            height (int): The height of the screen
        """
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
    

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
                pygame.draw.line(self.screen, color, (yVal[i],self.dim[1]//2),\
                                  (yVal[i] + stepsize, self.dim[1]//2), 1)

    def optimize(self):
        """
        Solves the nonlinear optimization problem to find the best control vectors for the trajectory
        """
        radius = [self.radius] * (len(self.obsState.keys()) + 1)
        bounds = [(0, self.dim[1]), (self.dim[0]//2 - self.dim[0]//5, self.dim[0]//2 + self.dim[0]//5), \
                  (0, self.maxVel),(-np.pi/4, np.pi/4),(None, None),(None, None)]
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
        pygame.draw.line(self.screen, ENV, (0, (roadCenter-roadWidth)),\
                          (self.dim[0], (roadCenter-roadWidth)), 1)
        pygame.draw.line(self.screen, ENV, (0, (roadCenter+roadWidth)), \
                         (self.dim[0], (roadCenter+roadWidth)), 1)

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
        save = True
        timestep = 0
        frames = []
        self.__initDisplay(self.dim[0], self.dim[1])
        while running and timestep < self.numStep:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            self.__refreshFrame(timestep)
            timestep = timestep + 1
            frame = pygame.surfarray.array3d(pygame.display.get_surface())
            frame = np.transpose(frame, (1, 0, 2))  # Transpose to match moviepy's format
            frames.append(frame)
            pygame.time.delay(100)
        pygame.quit()
        if save:
            clip = ImageSequenceClip(frames, fps=30)
            clip.write_videofile("output.mp4", codec="libx264")
