import numpy as np
import random 
import json


def calcRoad(dim):
    """
    Return information about the boundar and the center

    Returns:
        A tuple containing the center and the boundary of the lane
    """
    roadCenter = dim[1] // 2
    roadWidth = dim[1] // 5
    return (roadCenter, roadWidth)

def generateVehicle(maxVel, dim, road):
    """
    Generate a vehicle that we are optimizing for

    Returns:
        Initial and end conditions 
    """
    maxSize = 50
    v = random.uniform(maxVel-5, maxVel-1) * random.uniform(0.8,1)
    a = 0
    theta = 0
    w = 0
    
    # x,y position
    alpha = random.randint(0,1)
    x = random.randint(0, dim[1]//8)
    y = alpha * random.randint(road[0] + maxSize, road[0] + road[1] - maxSize)\
        + (1-alpha) * random.randint(road[0]-road[1]+maxSize, road[0] - maxSize)
    initial = [x, y, v, theta, w, a]
    end = [road[0], y, v, theta, w, a]
    return (initial, end)

def generateObstacle(numVehicle, maxVel, dim, road, radius):
    """
    Generate the obstacle vehicles on the two side of the road

    Args:
        numVehicle (int): The number of vehicle to generate

    Returns:
        Dictionaries containing information about each vehicle generated
    """
    obstacles = []
    for i in range(numVehicle):
        maxSize = 50
        # velocity, acceleration, angle, angular velocity
        v = random.uniform(maxVel-maxVel//2,maxVel - 5) 
        a = 0
        theta = 0
        w = 0
        # x,y position
        alpha = random.randint(0,1)
        x = random.randint(dim[1]//8 + 2*radius, dim[1] - dim[1]//4)
        y = alpha * random.randint(road[0] + maxSize, road[0] + road[1] -maxSize)\
            + (1-alpha) * random.randint(road[0]-road[1]+maxSize, road[0] - maxSize)
        feature = [x, y, v, theta, w, a]
        obstacles.append(feature)
    return obstacles

if __name__ == "__main__":
    numStep = 50
    maxVel = 25
    radius = 30
    dim = (580, 580)
    numObs = 3
    name = "env2"
    numStep = 100


    road = calcRoad(dim)
    vehicle_start, vehicle_end = generateVehicle(maxVel, dim, road)
    obstacles = generateObstacle(numObs, maxVel, dim, road, radius)

    output = {"max_velocity": maxVel,
              "num_Step": numStep,
              "radius": radius,
               "screenSize": dim,
               "road": road,
               "vehicle_start": vehicle_start,
               "vehicle_end": vehicle_end,
               "num_obstacles": numObs,
               "obstacles": obstacles,
               }
    with open(f"{name}.json", "w") as json_file:
        json.dump(output, json_file, indent=4) 