import numpy as np
from cyipopt import minimize_ipopt
import matplotlib.pyplot as plt

class AVP():
    def __init__(self, initial, end, bounds, numStep = 50, timeStep = 1, maxIter = 50):
        #initial/end ==> form of [x, y, v, theta, w, a]
        self.initial = initial 
        self.end = end
        self.numStep = numStep
        self.timeStep = timeStep
        self.maxIter = maxIter
        self.bounds = bounds * self.numStep
        self.initial_guess = np.zeros(len(self.initial) * self.numStep)

    def initializeTrajectory(self):
        # x = [x1, y1, v1, theta1, omega1, a1, ..., xN, yN, vN, thetaN, omegaN, aN]
        for i in range(self.timeStep):
            t = i / (self.timeStep - 1)
            self.initial_guess[6*i : 6*i + 4] = self.initial[0:4] + t * (self.end[0:4] - self.initial[0:4]) 
            self.initial_guess[6*i + 4 : 6*i + 6] = [self.initial[4], self.initial[5]] 

    # Define the objective function (e.g., minimize control effort)
    def objective(self, x):
        vel = -0.1 * x[3::6] #extract all the velocity
        omega = x[4::6]  # Extract all omega values
        accel = x[5::6]  # Extract all acceleration values
        return np.sum(vel + omega**2 + accel**2)  # Minimize control effort
    
    def constraints(self,x):
        cons = []
        # Initial conditions
        cons.extend(x[0:4] - self.initial[0:4])  # Enforce initial state
        
        # Final conditions
        cons.extend(x[-6:-2] - self.end[0:4])  # Enforce final state
        for i in range(self.numStep - 1):
            x1, y1, v1, theta1, omega1, a1 = x[6*i : 6*(i+1)]
            x2, y2, v2, theta2, omega2, a2 = x[6*(i+1) : 6*(i+2)]

            # Kinematics x_{k+1} = x_k + v_k * cos(theta_k) * dt
            cons.append(x2 - (x1 + v1 * np.cos(theta1) * self.timeStep))
            cons.append(y2 - (y1 + v1 * np.sin(theta1) * self.timeStep))
            cons.append(v2 - (v1 + a1 * self.timeStep))
            cons.append(theta2 - (theta1 + omega1 * self.timeStep))
        return np.array(cons)

    def forward(self):
        result = minimize_ipopt(
            self.objective,
            self.initial_guess,
            bounds=self.bounds,
            constraints={'type': 'eq', 'fun': self.constraints},
            options={'disp': 5, 'max_iter':self.maxIter}
        )
        solution = result.x
        return solution 


def visualizeSolutionPosition(solution):
    x_pos = solution[0::6]  # Every 6th element starting from index 0
    y_pos = solution[1::6]  # Every 6th element starting from index 1
    vx = solution[2::6]     # Every 6th element starting from index 2
    vy = solution[3::6]     # Every 6th element starting from index 3
    ax = solution[4::6]     # Every 6th element starting from index 4
    ay = solution[5::6]     # Every 6th element starting from index 5

    # Time vector
    time = np.linspace(0, 1*50, 50)  # T is the total time

    plt.figure(figsize=(8, 6))
    plt.plot(x_pos, y_pos, '-o', label='Optimized Trajectory')
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
    initial = np.array([0, 0, 0.5, 0, 0, 0])
    end = np.array([15, 0 , 0.5, 0, 0, 0])

    model = AVP(initial, end, bounds)
    sol = model.forward()
    visualizeSolutionPosition(sol)