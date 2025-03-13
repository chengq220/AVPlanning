from environment import environemnt
import numpy as np

if __name__ == "__main__":
    env = environemnt([580,580],numVehicles=3)
    env.runGame()