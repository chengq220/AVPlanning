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
    center, width = road
    v = random.uniform(maxVel-5, maxVel-1) * random.uniform(0.8,1)
    a = 0
    theta = 0
    w = 0
    
    # x,y position
    alpha = random.randint(0,1)
    x = random.randint(1.1 * radius, dim[1]//6)
    y = center + width//2 - radius if alpha == 0 else center - width//2 + radius
    initial = [x, y, v, theta, w, a]
    end = [road[0], y, v, theta, w, a]
    return (initial, end)

def generateObstacle(numVehicle, maxVel, dim, road, radius, vehicle_start):
    """
    Generate the obstacle vehicles on the two side of the road

    Returns:
        Dictionaries containing information about each vehicle generated
    """
    center, width = road
    obstacles = []
    # x coordinate of the target vehicle
    target_x = vehicle_start[0]
    
    for _ in range(numVehicle):
        # velocity, acceleration, angle, angular velocity
        v = random.uniform(maxVel-maxVel//2,maxVel - 5) 
        a = random.uniform(1,2) 
        theta = 0
        w = 0

        # x,y position
        alpha = random.randint(0,1)
        if alpha == 0:
            x = random.randint(0, target_x-radius)
        else:
            x = random.randint(target_x+radius, dim[0])
        y = center + width//2 - radius if alpha == 0 else center - width//2 + radius
        feature = [x, y, v, theta, w, a]
        obstacles.append(feature)
    return obstacles

if __name__ == "__main__":
    name = "env4"
    numStep = 50
    maxVel = 25
    radius = 30
    dim = (580, 580)
    numObs = 3
    numStep = 100
    eps = 0.1

    road = calcRoad(dim)
    vehicle_start, _ = generateVehicle(maxVel, dim, road)
    obstacles = generateObstacle(numObs, maxVel, dim, road, radius, vehicle_start)

    output = {"max_velocity": maxVel,
              "num_Step": numStep,
              "radius": radius,
               "screenSize": dim,
               "eps": eps,
               "road": road,
               "vehicle_start": vehicle_start,
               "num_obstacles": numObs,
               "obstacles": obstacles,
               }
    with open(f"environment/{name}.json", "w") as json_file:
        json.dump(output, json_file, indent=4) 