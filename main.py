from environment import environemnt

def startSimulation():
    """
    Starts the simulation
    """
    env = environemnt("environment/env1.json")
    env.optimize()
    env.runGame()

if __name__ == "__main__":
    startSimulation()