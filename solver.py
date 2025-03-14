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
        n = 2
        m = 0  
        super().__init__(n, m)

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

# Initial guess
x0 = np.array([-1.2, 1.0])

# Instantiate the problem and solve
problem = Rosenbrock()
solver = problem
sol, info = solver.solve(x0)

# Display results
print("Optimal solution:", sol)
