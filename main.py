from environment import environemnt

def startSimulation():
    """
    Starts the simulation
    """
    env = environemnt("environment/env1.json")
    env.optimize(maxIter=20)
    env.runGame(save = "env1.mp4")

if __name__ == "__main__":
    startSimulation()