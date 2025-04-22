import pygame 
import numpy as np
import random
from source.newEnv.solver import AVP
from scipy.interpolate import make_interp_spline
import json
from moviepy.video.io.ImageSequenceClip import ImageSequenceClip

PLAYER = (0, 0, 255) #blue color
OBSTACLE = (0, 255, 0) #green color
ENV = (255, 0, 0) #red color
SCREEN = (0, 0, 0) #black color

class environemnt():
    """
    A class that simulates the environment for the vehicle planning problem 
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
        self.vehicle = np.array(data["vehicle_start"])
        self.eps = data["eps"]
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


    def __generate_spline(self):
        pt = np.array([0, 290, 580])
        midY = np.array([250, 430, 250])
    
        screen_height = self.dim[1]  
        midY = screen_height - midY  
        
        splMid = make_interp_spline(pt, midY, k=2)
        return splMid

    
    def __drawDashedLine(self, color, stepsize = 20):
        """
        Draw the dashed center line of the road

        Args:
            color (tuple): RGB value of the line
            stepSize (int): the size of the space in the dashed line 
        """
        midFunc = self.__generate_spline()
        current_x = 0
        
        while current_x < self.dim[0]:
            dash_end = current_x + stepsize
            if dash_end > self.dim[0]:
                dash_end = self.dim[0]
            
            pygame.draw.line(self.screen, color,
                            (current_x, int(midFunc(current_x))),
                            (dash_end, int(midFunc(dash_end))),
                            1)
            current_x += 2* stepsize

    def load(self, model):
        """
        Loads the optimized trajectory (if it exists)
        Args:
            model (string) : The directory where model is saved
        """ 
        data = np.load(model)
        self.sol = data['arr1']
        self.obstacleTrajectory = data['arr2']

    def optimize(self, name, maxIter = 10):
        """
        Solves the nonlinear optimization problem to find the best control vectors for the trajectory
        Args: 
            name (string)    : The name of the save location
            maxIter (integer): The number of iteration for the optimizer 
        """
        radius = [self.radius] * (len(self.obsState.keys()) + 1)
        #[x, y, v, theta, omega, accel]
        bounds = [(None, None), (self.dim[0]//2 - self.dim[0]//5 + self.radius, self.dim[0]//2 + self.dim[0]//5 -self.radius), \
                  (5, self.maxVel),(-1*np.pi/4, np.pi/4),(-1*np.pi/6, np.pi/6),(0, 4)]
        parameters = [self.road, self.vehicle, self.obsState, radius, bounds, self.eps]
        model = AVP(parameters, numStep=self.numStep, maxIter=maxIter)
        sol = model.forward()
        self.sol = (sol[0::6], sol[1::6])
        np.savetxt(name + ".txt", np.column_stack((sol[0::6], sol[1::6], sol[2::6],sol[3::6],sol[4::6], sol[5::6])), 
           fmt='%.6f',  # Saves with 6 decimal places
           header='X,         Y,         V,        Theta,     Omega,      Accel', 
           comments='')
        self.obstacleTrajectory = model.obstacles
        np.savez(name + ".npz", arr1=self.sol, arr2=self.obstacleTrajectory)
    
    def __refreshFrame(self, timestep):
        """
        Refreshes the screen with updated values
        Args: 
            timestep (integer): The timestep for each update
        """
        self.screen.fill(SCREEN) 

        # Update the desire vehicle
        vx, vy = self.sol
        pygame.draw.circle(self.screen, PLAYER, (vx[timestep],vy[timestep]), self.radius)
        
        # Draw the road for each frame
        self.__drawDashedLine(ENV, stepsize=10)
        roadCenter, roadWidth = self.road
        midFunc = self.__generate_spline()
        x_vals = np.linspace(0, self.dim[0], 100) 
        top = midFunc(x_vals) + roadWidth
        bot = midFunc(x_vals) - roadWidth
        for i in range(len(x_vals)-1):
            pygame.draw.line(self.screen, ENV, 
                            (x_vals[i], int(top[i])),
                            (x_vals[i+1], int(top[i+1])), 
                            1)
            
            pygame.draw.line(self.screen, ENV,
                            (x_vals[i], int(bot[i])),
                            (x_vals[i+1], int(bot[i+1])),
                            1)

        # Update the obstacles 
        for idx in range(self.obstacleTrajectory.shape[0]):
            obs_x = self.obstacleTrajectory[idx, 6*timestep]
            obs_y = self.obstacleTrajectory[idx, 6*timestep + 1]
            pygame.draw.circle(self.screen, OBSTACLE, (obs_x,obs_y), self.radius)
        
        pygame.display.update()

    def runGame(self, save = "output1.mp4", saveVid = True):
        """
        Run the simulation/game
        """
        running = True
        timestep = 0
        frames = []
        manual = False
        self.__initDisplay(self.dim[0], self.dim[1])
        while running and timestep < self.numStep:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    # for manual control of the simulation
                    if event.key == pygame.K_SPACE:
                        print("Space bar pressed!")
                        manual = True
            
            self.__refreshFrame(timestep)
            timestep = timestep + 1
            frame = pygame.surfarray.array3d(pygame.display.get_surface())
            frame = np.transpose(frame, (1, 0, 2))  # Transpose to match moviepy's format
            frames.append(frame)
            if not manual: 
                pygame.time.delay(100)
            else:
                # wait for user to press next
                waiting = True
                while waiting:
                    for event in pygame.event.get(): 
                        if event.type == pygame.QUIT:
                            running = False
                            waiting = False
                        elif event.type == pygame.KEYDOWN:
                            if event.key == pygame.K_RIGHT:
                                print("Right arrow key pressed!")
                                waiting = False
                            if event.key == pygame.K_SPACE:
                                print("Space bar pressed!")
                                manual = False
                                waiting = False
        pygame.quit()
        if saveVid: #Save the video to device
            clip = ImageSequenceClip(frames, fps=15)
            clip.write_videofile(save, codec="libx264")
