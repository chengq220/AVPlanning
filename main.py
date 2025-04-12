from source.environment import environemnt
import os

def startSimulation(env, save, iter = 10):
    """
    Starts the simulation
    """
    filename = os.path.basename(env).split(".")[0]
    env = environemnt(env)
    model = os.path.join("save", filename + ".npz")
    if os.path.exists(model): #Get the model if the model exist
        env.load(model)
    else:
        env.optimize(name = model, maxIter=iter)
    env.runGame(save=save)

if __name__ == "__main__":
    startSimulation("environment/env4.json", "output/testing.mp4")