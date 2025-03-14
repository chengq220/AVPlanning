from environment import environemnt
import numpy as np

def startSimulation(dim, timeStep = 1, numVehicles = 3, maxVelocity = 2):
    env = environemnt([dim, dim], timeStep = timeStep, maxVel = maxVelocity)
    env.runGame()

if __name__ == "__main__":
    startSimulation(580)