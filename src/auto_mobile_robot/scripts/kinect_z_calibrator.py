#!/usr/bin/env python3
import numpy as np
from scipy.optimize import least_squares

# # Data points (original data)
# x_data = np.array([420, 685, 825, 855, 970], dtype=np.float64)
# f_data = np.array([0.4, 0.8, 1.2, 1.8, 2.8], dtype=np.float64)

# Data points (talyees)
x_data = np.array([450, 650,  730, 890, 990],  dtype=np.float64)
f_data = np.array([0.5, 0.75, 1.0, 1.6, 3.35], dtype=np.float64)

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
print(f"        a, b, c, p, q = {a:.22f}, {b:.22f}, {c:.22f},  {p:.22f}, {q:.22f}")
print(f"f(x) = {a} * {b}^({c}x+({p})) + {q}") # a:.7f
# a, b, c, p, q = 0.0099409, 1.0764918, 0.0774622,  -0.0166755, 0.2940652

x_test = int(input("Enter x value to test: "))
print(f"f({x_test}) = {model(result.x, x_test)}")
