from source.environment import environemnt
import os

def startSimulation(env, save, iter = 10, saveVid = True):
    """
    Starts the simulation
    """
    filename = os.path.basename(env).split(".")[0]
    env = environemnt(env)
    model = os.path.join("save", filename)
    # Get the model if the model exist
    if os.path.exists(model+ ".npz"): 
        print("Loading model")
        env.load(model+ ".npz")
        saveVid = False
    else:
        env.optimize(name = model, maxIter=iter)
    env.runGame(save=save, saveVid = saveVid)
    

if __name__ == "__main__":
    startSimulation("environment/demo_gen.json", "output/demo_gen.mp4")