import numpy as np
from cyipopt import minimize_ipopt
import matplotlib.pyplot as plt

class AVP():
    """
    A class that solves that path planning optimization problem
    """
    def __init__(self, parameters, numStep = 100, timeStep = 0.5, maxIter = 50, screensize = 580):
        """
        Initialize the solver
        """
        # Setting up the environment variable
        self.numStep = numStep
        self.timeStep = timeStep
        self.maxIter = maxIter
        self.road = parameters[0]
        self.bounds = parameters[4] * self.numStep
        self.screensize = screensize

        self.eps = parameters[5]
        # The radius of each vehicle (index 0 is the vehicle we are optimizing for)
        self.radius = parameters[3]

        #initial ==> form of [x, y, v, theta, w, a]
        self.initial = parameters[1]
        self.initial_guess = self.initializeTrajectory()
        # O is in the form of a dictionary of [x, y, v, theta, w, a]
        self.obstacles = self.obstacleTrajectory(parameters[2])

    def initializeTrajectory(self):
        """
        Predict the trajectories of the desired vehicles base on initial guess
        Args: 
            None
        Returns:
            The initial guess of the trajectory
        """
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
        """
        Predict the trajectories of the obstacle vehicles base on its static values
        Args: 
            obstacle (dict): containing the information about the obstacles
        Returns:
            An array containing the trajectories taken by the obstacle vehicles 
        """
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

    def objective(self, x):
        """
        Predict the trajectories of the desired vehicles base on initial guess
        Args: 
            x (list): List containing the information about the trajectories 
        Returns:
            The cost of the trajectories 
        """
        # _, _, vb, thetab, omegab, accelb = self.bounds[0:6]
        velocities = x[2::6]
        theta = x[3::6]
        omega = x[4::6]
        accel = x[5::6]
        
        #Target velocity components
        vel_x = velocities * np.cos(theta)
        vel_x_cost = -0.4 * np.sum(vel_x**2)

        vel_y = velocities * np.sin(theta)
        vel_y_cost = 0.5 * np.sum(vel_y**2)
        
        # Control effort terms
        omega_cost = 0.7 * np.sum(omega**2)  # Penalize large steering
        accel_cost = 0.1 * np.sum(accel**2)  # Penalize large acceleration

        # change in omega/accel
        dtheta = 0.5 * np.sum(np.diff(theta) ** 2)
        daccel = 0.5 * np.sum(np.diff(accel) ** 2)

        # Total cost function
        J = vel_x_cost + vel_y_cost + omega_cost + accel_cost + dtheta + daccel
        return J

    def equality_constraints(self,x):
        """
        Define the equality constraints for the program
        Args: 
            x (list): List containing the information about the trajectories 
        Returns:
            A list of equality constraints
        """
        cons = []
        # Enforce initial state
        cons.extend(x[0:2] - self.initial[0:2])  
        for i in range(self.numStep - 1):
            x1, y1, v1, theta1, omega1, a1 = x[6*i : 6*(i+1)]
            x2, y2, v2, theta2, _, _ = x[6*(i+1) : 6*(i+2)]

            # Kinematics y_{k+1} = y_k + v_k * sin(theta_k) * dt
            # Constraint to ensures that the state is consistent with the kinematic model
            cons.append(y2 - (y1 + v1 * np.sin(theta1) * self.timeStep))
            cons.append(v2 - (v1 + a1 * self.timeStep))
            cons.append(theta2 - (theta1 + omega1 * self.timeStep))
        return np.array(cons)
    
    def inequality_constraints(self, x):
        """
        Define the inequality constraints for the program
        Args: 
            x (list): List containing the information about the trajectories 
        Returns:
            A list of inequality constraints
        """
        cons = []
        # The collision constraint
        for idx in range(self.obstacles.shape[0]):
            curObstacle = self.obstacles[idx]
            r1, r2 = self.radius[0], self.radius[idx+1]
            for k in range(self.numStep):
                distance_sq = np.linalg.norm(x[6*k:6*k+2] - curObstacle[6*k:6*k+2]) ** 2
                # Ensure that each vehicle has some distance between them
                cons.append(distance_sq - (1 + self.eps)*(r1 + r2) ** 2)
        
        yPos = x[1::6]
        topLaneBound = self.road[0] + self.road[1] - self.radius[0]
        botLaneBound = self.road[0] - self.road[1] + self.radius[0]
        # Constraints for the lane
        for pos in yPos:
            cons.append(topLaneBound - pos)   
            cons.append(pos - botLaneBound) 

        return np.array(cons)

    def forward(self):
        """
        Runs the optimization library to solve the optimization formulation
        Args: 
            None
        Returns:
            The solution the the path planning program
        """
        options = {
            'disp': 5,
            'max_iter': self.maxIter, 
            'tol': 1e-6,
            'constr_viol_tol': 1e-6,
            'acceptable_tol': 1e-4
        }
        # Setup the nonlinear solver
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
