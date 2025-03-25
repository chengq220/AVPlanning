from environment import environemnt

def startSimulation():
    """
    Starts the simulation
    """
    env = environemnt("environment/env3.json")
    env.optimize(maxIter=20)
    env.runGame()

if __name__ == "__main__":
    startSimulation()