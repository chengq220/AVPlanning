import numpy as np
from cyipopt import Problem

class AVP(Problem):
    """
    A nonlinear optimization solver setup to solve the Autonomous Vehicle Planning Problem
    """
    def __init__(self, variables, constraints):
        """
        Initialize the environment

        Args:
            variables (int): The number of variables
            constraints (int): The number of constraints
        """
        self.var = variables
        self.con = constraints 
        super().__init__(self.var, self.con)

    def objective(self, x):
        """Compute the objective function value"""
        return (1 - x[0])**2 + 100 * (x[1] - x[0]**2)**2

    def gradient(self, x):
        """Compute the gradient of the objective function"""
        df_dx0 = -2 * (1 - x[0]) - 400 * x[0] * (x[1] - x[0]**2)
        df_dx1 = 200 * (x[1] - x[0]**2)
        return np.array([df_dx0, df_dx1])

    def constraints(self, x):
        """Return constraint function values (empty in this case)"""
        # the car is within the boundary of the lanes

        return np.array([])

    def jacobian(self, x):
        """Return the Jacobian of the constraints (empty in this case)"""
        return np.array([])

    def hessianstructure(self):
        """Return the sparsity structure of the Hessian"""
        return np.tril_indices(2)

    def hessian(self, x, lagrange, obj_factor):
        """Return the Hessian matrix of the Lagrangian"""
        H = np.zeros((2, 2))
        H[0, 0] = obj_factor * (2 - 400 * (x[1] - 3 * x[0]**2))
        H[0, 1] = obj_factor * (-400 * x[0])
        H[1, 0] = H[0, 1]
        H[1, 1] = obj_factor * 200
        return H[self.hessianstructure()]
    
    def __objectiveFunction(x, u, r):
        """
        The cost evaluating function

        Args:
            x (tuple): information about x
            u (vector): control vector
            r (matrix): weight matrix

        Returns: 
            The cost at that specific time step
        """
        vel = x[4]
        
        #incentive faster speed and penalize sharp changes in the control
        cost = -0.1 * vel + u.T @ r @ u
        return cost
        
    def __collisionConstraints(x, y):
        """
        Check for whether the two object are colliding
        Res = (x-y)^2 - (x_radius - y_radius)^2
        1. res > 0 ==> no overlap
        2. res = 0 ==> objects are touching
        2. res < 0 ==> objects are overlapping

        Args:
            x (tuple): information about x, y, radius of x
            y (tuple): information about x, y, radius of y

        Returns: 
            Integer indicating whether there is an overlap or not
        """
        (x1, x2, _, _, r1) = x
        (y1, y2, _, _, r2)  = y
        res = (x1 - y1)**2 + (x2- y2)**2 - (r1 - r2)**2
        return res
    
    def __boundaryConstraints(self, A, B, C):
        """
        Uses half space representation for the boundary lines in the form 
        A^T B + C 

        Args:
            A (Vector): A normal vector to the boundary lane
            B (Vector): The X,Y position of the vehicle
            C (Integer): The offset of the boundary lane
        """
        val = np.dot(A, B) + C
        return val

if __name__ == "__main__":
    # Initial guess
    x0 = np.array([-1.2, 1.0])

    # Instantiate the problem and solve
    problem = AVP(2,3)
    solver = problem
    sol, info = solver.solve(x0)

    # Display results
    print("Optimal solution:", sol)
