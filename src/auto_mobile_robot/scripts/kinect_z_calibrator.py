#!/usr/bin/env python3


# for quadratic model:
'''
import numpy as np
from scipy.optimize import least_squares

# Data points (talyees)
x_data = np.array([450, 650,  730, 890, 990],  dtype=np.float64)
f_data = np.array([0.5, 0.75, 1.0, 1.6, 3.35], dtype=np.float64)

# Define the model
def model(params, x):
    a, b, c = params
    return a * x**2 + b * x + c

# Residual function
def residuals(params, x, f):
    return f - model(params, x)

# Initial guesses for [a, b, c, p, q]
initial_guess = [1.0, 1.01, 0.001]

# Solve for parameters
result = least_squares(residuals, initial_guess, args=(x_data, f_data))
a, b, c = result.x

# Output the solution
print(f"        a, b, c = {a:.22f}, {b:.22f}, {c:.22f}")
print(f"f(x) = {a} * x^2 + {b} * x + {c} ") # a:.7f
# a, b, c, p, q = 0.0099409, 1.0764918, 0.0774622,  -0.0166755, 0.2940652

x_test = int(input("Enter x value to test: "))
print(f"f({x_test}) = {model(result.x, x_test)}")

'''




# for exponential model:
'''
import numpy as np
from scipy.optimize import least_squares

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
'''




# for trig (tan) equation




# for exponential model:

import numpy as np
from scipy.optimize import least_squares

# Data points (talyees)
x_data = np.array([706,   781,  856,  923,  976, 1000, 1007, 1014, 1020, 1028, 1053, 1056],    dtype=np.float64)
f_data = np.array([0.94, 1.13, 1.43, 2.08, 2.90, 3.65, 3.90, 4.22, 4.57, 5.00, 8.00, 8.60],    dtype=np.float64)

# Define the model
def model(params, x):
    a, b, c, d= params
    return a * np.tan(x/b + c) + d

# Residual function
def residuals(params, x, f):
    return f - model(params, x)

# Initial guesses for [a, b, c, d]
# 0.076 * tan( x/4614 + 1.333 ) + 0.03
initial_guess = [0.076, 4614, 1.333, 0.03]

# Solve for parameters
result = least_squares(residuals, initial_guess, args=(x_data, f_data))
a, b, c, d = result.x

# Output the solution
print(f"        self.a, self.b, self.c, self.d = {a:.22f}, {b:.22f}, {c:.22f}, {d:.22f}")
print(f"f(x) = {a}*tan((x/{b}) + {c}) + {d}") # a:.7f

x_test = int(input("Enter x value to test: "))
print(f"f({x_test}) = {model(result.x, x_test)}")
