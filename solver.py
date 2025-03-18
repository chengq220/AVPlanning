import numpy as np
from cyipopt import minimize_ipopt
import matplotlib.pyplot as plt

class AVP():
    def __init__(self, X, O, radius, bounds, numStep = 50, timeStep = 1, maxIter = 50, screensize = 580):
        self.numStep = numStep
        self.timeStep = timeStep
        self.maxIter = maxIter
        self.bounds = bounds * self.numStep
        self.screensize = screensize

        # The radius of each vehicle (index 0 is the vehicle we are optimizing for)
        self.radius = radius
        
        #initial/end ==> form of [x, y, v, theta, w, a]
        self.initial, self.end = X
        self.initial_guess = self.initializeTrajectory()

        # O is in the form of a dictionary of [x.y,v,theta,w,a]
        self.obstacles = self.obstacleTrajectory(O)


    # Predict the trajectories of the desired vehicles base on initial guess
    def initializeTrajectory(self):
        initial_guess = np.zeros(len(self.initial) * self.numStep)
        # x = [x1, y1, v1, theta1, omega1, a1, ..., xN, yN, vN, thetaN, omegaN, aN]
        for i in range(self.numStep):
            t = i / (self.numStep - 1)
            initial_guess[6*i : 6*i + 4] = self.initial[0:4] + t * (self.end[0:4] - self.initial[0:4]) 
            initial_guess[6*i + 4 : 6*i + 6] = [self.initial[4], self.initial[5]] 
        return initial_guess

    # Predict the trajectories of the obstacle vehicles
    def obstacleTrajectory(self, obstacle):
        traj = []
        for idx, feature in obstacle.items():
            cur = np.zeros(len(self.initial) * self.numStep)
            for i in range(self.numStep):
                # t = i / (self.numStep - 1)
                # Simple kinematic setup for straight horizontal motion
                if i==0:
                    newX = feature[0] +  feature[2] * np.cos(feature[3])
                    newY = feature[1] +  feature[2] * np.sin(feature[3]) 
                else:
                    newX = cur[6*(i-1)] + cur[2] * np.cos(cur[3])
                    newY = cur[6*(i-1)+1] + cur[2] * np.sin(cur[3]) 
                cur[6*i : 6*i + 2] = [newX, newY]  # Update x and y positions
                cur[6*i + 2 : 6*i + 6] = [feature[2], feature[3], feature[4], feature[5]]  # Keep other states constant
            traj.append(cur)
        return np.array(traj)

    # Define the objective function (e.g., minimize control effort)
    def objective(self, x):
        vel = -0.1 * x[3::6] #extract all the velocity
        omega = x[4::6] 
        accel = x[5::6]  
        return np.sum(vel + 0.1 * omega**2 + 0.5 * accel**2) 
    
    # Defines the equaltiy constraint for the problem
    def equality_constraints(self,x):
        cons = []
        # Initial conditions
        cons.extend(x[0:4] - self.initial[0:4])  # Enforce initial state

        # Final conditions
        cons.extend(x[-6:-2] - self.end[0:4])  # Enforce final state
        for i in range(self.numStep - 1):
            x1, y1, v1, theta1, omega1, a1 = x[6*i : 6*(i+1)]
            x2, y2, v2, theta2, _, _ = x[6*(i+1) : 6*(i+2)]

            # Kinematics x_{k+1} = x_k + v_k * cos(theta_k) * dt
            # Constraint to ensures that the state is consistent with the kinematic model
            cons.append(x2 - (x1 + v1 * np.cos(theta1) * self.timeStep))
            cons.append(y2 - (y1 + v1 * np.sin(theta1) * self.timeStep))
            cons.append(v2 - (v1 + a1 * self.timeStep))
            cons.append(theta2 - (theta1 + omega1 * self.timeStep))
        return np.array(cons)
    
    # Defines the inequality constraints for the problems
    def inequality_constraints(self, x):
        cons = []
        # The collision constraint
        for idx in range(self.obstacles.shape[0]):
            curObstacle = self.obstacles[idx]
            r1, r2 = self.radius[0], self.radius[idx+1]
            for k in range(self.numStep):
                distance_sq = np.dot((x[6*k:6*k+2] - curObstacle[6*k:6*k+2]), 
                                    (x[6*k:6*k+2] - curObstacle[6*k:6*k+2]))
                collision = distance_sq - (r1 + r2) ** 2
                cons.append(collision)
        return np.array(cons)

    # Runs the optimization library to solve the optimization formulation
    def forward(self):
        result = minimize_ipopt(
            self.objective,
            self.initial_guess,
            bounds=self.bounds,
            constraints=[
                {'type': 'eq', 'fun': self.equality_constraints},  # Equality constraints
                {'type': 'ineq', 'fun': self.inequality_constraints}  # Inequality constraints
            ],
            options={'disp': 5, 'max_iter':self.maxIter}
        )
        solution = result.x
        return solution 


def visualizeSolutionPosition(solution, obstacles):
    x_pos = solution[0::6]  # Every 6th element starting from index 0
    y_pos = solution[1::6]  # Every 6th element starting from index 1

    plt.figure(figsize=(8, 6))
    plt.plot(x_pos, y_pos, '-o', label='Optimized Trajectory')
    
    # Plot obstacle trajectories
    for idx in range(obstacles.shape[0]):
        obs_x = obstacles[idx, 0::6]
        obs_y = obstacles[idx, 1::6]
        plt.plot(obs_x, obs_y, '--', label=f'Obstacle {idx+1}')
    
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Optimized Path Trajectory')
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    #define the bounds and conditions
    bounds = [(0, None), (0, None),(0, 3),(-np.pi/2, np.pi/2),\
        (None, None),(None, None)]
    initial = np.array([0, 0, 1, 0, 0, 0])
    end = np.array([15, 0, 1, 0, 0, 0])
    radius = [2,3]
    obstacle = dict()
    for i in range(1):
        obstacle[i] = np.array([6,3,0.6,0,0,0])

    model = AVP((initial, end), obstacle, radius, bounds)
    sol = model.forward()
    print(model.equality_constraints(sol))
    visualizeSolutionPosition(sol, model.obstacles)