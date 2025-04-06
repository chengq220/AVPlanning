from source.environment import environemnt

def startSimulation(env, save, iter = 10):
    """
    Starts the simulation
    """
    env = environemnt(env)
    env.optimize(maxIter=iter)
    env.runGame(save=save)

if __name__ == "__main__":
    startSimulation("environment/env1.json", "output/testing.mp4")