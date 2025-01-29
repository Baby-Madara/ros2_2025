#!/usr/bin/env python3
import numpy as np
from scipy.optimize import least_squares

# Data points (original data)
x_data = np.array([420, 685, 825, 855, 970], dtype=np.float64)
f_data = np.array([0.4, 0.8, 1.2, 1.8, 2.8], dtype=np.float64)

# Data points (talyees)
x_data = np.array([435, 690, 750, 835, 870, 900, 980], dtype=np.float64)
f_data = np.array([0.4, 0.8, 1.0, 1.2, 1.4, 1.7, 3.35], dtype=np.float64)

# Define the model
def model(params, x):
    a, b, c, p, q = params
    return a * b ** (c * x + p) + q

# Residual function
def residuals(params, x, f):
    return f - model(params, x)

# Initial guesses for [a, b, c, p, q]
initial_guess = [1.0, 1.01, 0.001, 0.0, 0.0]

# Solve for parameters
result = least_squares(residuals, initial_guess, args=(x_data, f_data))
a, b, c, p, q = result.x

# Output the solution
print(f"        a, b, c, p, q = {a:.7f}, {b:.7f}, {c:.7f},  {p:.7f}, {q:.7f}")
print(f"f(x) = {a:.7f} * {b:.7f}^({c:.7f} * x+({p:.7f})) + {q:.7f}")
# a, b, c, p, q = 0.0099409, 1.0764918, 0.0774622,  -0.0166755, 0.2940652

