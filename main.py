from environment import environemnt
import numpy as np

def startSimulation(dim, numSteps = 50, numVehicles = 1, maxVelocity = 2):
    """
    Starts the simulation
    """
    env = environemnt([dim, dim], numStep = numSteps,numVehicles=numVehicles, maxVel = maxVelocity)
    env.optimize()
    env.runGame()

if __name__ == "__main__":
    startSimulation(580, numVehicles= 3)