import numpy as np
from cyipopt import minimize_ipopt
import matplotlib.pyplot as plt

class AVP():
    def __init__(self, road, X, O, radius, bounds, numStep = 100, timeStep = 0.5, maxIter = 50, screensize = 580):
        # Setting up the environment variable
        self.numStep = numStep
        self.timeStep = timeStep
        self.maxIter = maxIter
        self.bounds = bounds * self.numStep
        self.screensize = screensize
        self.road = road
        
        # The radius of each vehicle (index 0 is the vehicle we are optimizing for)
        self.radius = radius

        #initial ==> form of [x, y, v, theta, w, a]
        self.initial = X

        self.initial_guess = self.initializeTrajectory()

        # O is in the form of a dictionary of [x, y,v,theta,w,a]
        self.obstacles = self.obstacleTrajectory(O)

    # Predict the trajectories of the desired vehicles base on initial guess
    def initializeTrajectory(self):
        initial_guess = np.zeros(len(self.initial) * self.numStep)
        for i in range(self.numStep):
            t = i / (self.numStep - 1)
            if i == 0:
                # Use initial conditions
                initial_guess[6*i:6*i+6] = self.initial
            else:
                # Propagate state using kinematics
                prev = initial_guess[6*(i-1):6*i]
                initial_guess[6*i] = (prev[0] + self.timeStep * prev[2] * np.cos(prev[3]))%self.screensize
                initial_guess[6*i+1] = prev[1] + self.timeStep * prev[2] * np.sin(prev[3])%self.screensize
                initial_guess[6*i+2] = prev[2]  # Keep velocity constant initially
                initial_guess[6*i+3] = 0.1 * np.cos(t * np.pi)  # Small steering
                initial_guess[6*i+4] = 0  # Zero angular velocity
                initial_guess[6*i+5] = 1.2  # Zero acceleration
        return initial_guess
    
    # Predict the trajectories of the obstacle vehicles
    def obstacleTrajectory(self, obstacle):
        traj = []
        for idx, feature in obstacle.items():
            cur = np.zeros(len(self.initial) * self.numStep)
            for i in range(self.numStep):
                # Simple kinematic setup for straight horizontal motion
                if i==0:
                    newVel = feature[2]
                    newX = feature[0]
                    newY = feature[1]
                else:
                    newVel = cur[6*(i-1) + 2] + self.timeStep * cur[6*(i-1)+ 5] 
                    newX = cur[6*(i-1)] + self.timeStep * cur[2] * np.cos(cur[3])
                    newY = cur[6*(i-1)+1] + self.timeStep * cur[2] * np.sin(cur[3]) 
                newX = newX % self.screensize
                newY = newY % self.screensize
                cur[6*i : 6*i + 2] = [newX, newY]  # Update x and y positions
                cur[6*i + 2 : 6*i + 6] = [newVel, feature[3], feature[4], feature[5]] 
            traj.append(cur)
        return np.array(traj)

    # Define the objective function (e.g., minimize control effort)
    def objective(self, x):
        velocities = x[2::6]
        theta = x[3::6]
        omega = x[4::6]
        accel = x[5::6]
        
        # Target velocity components
        vel_cost = -0.2 * np.sum(velocities**2)
        
        # Control effort terms
        omega_cost = 0.5 * np.sum(omega**2)  # Penalize large steering
        accel_cost = 0.1 * np.sum(accel**2)  # Penalize large acceleration

        #change in omega/accel
        dtheta = 0.5 * np.sum(np.diff(theta) ** 2)
        daccel = 0.5 * np.sum(np.diff(accel) ** 2)
        
        return vel_cost + omega_cost + accel_cost + dtheta + daccel
    
    # Defines the equaltiy constraint for the problem
    def equality_constraints(self,x):
        cons = []
        # Enforce initial state
        cons.extend(x[0:2] - self.initial[0:2])  
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
                distance_sq = np.linalg.norm(x[6*k:6*k+2] - curObstacle[6*k:6*k+2]) ** 2
                cons.append(distance_sq - (r1 + r2) ** 2)

        # Theta bound (constraint -pi/4 <= theta <= pi/4)
        theta_n = x[3::6]
        for idx in range(len(theta_n)):
            cons.append(theta_n[idx] + np.pi/4)
            cons.append(np.pi/4 - theta_n[idx])

        # roadCenter, roadWidth = self.road
        # topLane = np.array([0,1])
        # b_top = roadCenter + roadWidth
        # botLane = np.array([0,-1])
        # b_bot = roadCenter - roadWidth

        # Lane constraint
        # for k in range(self.numStep):
        #     X = x[6*k:6*k+6]
        #     cons.append(self.lane_constraint(X, topLane, b_top, self.radius[0]))
        #     cons.append(self.lane_constraint(X, botLane, b_bot, self.radius[0]))

        return np.array(cons)

    # Makes sure that the vehicle remains within the lane
    # def lane_constraint(self, X, a, b, r):
    #     return np.dot(a, X[:2] - a*r) - b

    # Runs the optimization library to solve the optimization formulation
    def forward(self):
        options = {
            'disp': 5,
            'max_iter': self.maxIter, 
            'tol': 1e-6,
            'constr_viol_tol': 1e-6,
            'acceptable_tol': 1e-4
        }
        result = minimize_ipopt(
            self.objective,
            self.initial_guess,
            bounds=self.bounds,
            constraints=[
                {'type': 'eq', 'fun': self.equality_constraints},  # Equality constraints
                {'type': 'ineq', 'fun': self.inequality_constraints}  # Inequality constraints
            ],
            options=options
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
