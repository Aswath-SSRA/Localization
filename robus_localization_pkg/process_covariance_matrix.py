# -*- coding: utf-8 -*-

"""
# Continuous White Noise Model
"""
from sympy import (init_printing, Matrix, MatMul,
integrate, symbols)
init_printing(use_latex='mathjax')
dt, phi = symbols('\Delta{t} \Phi_s')
F_k = Matrix([[1, dt, dt**2/2],
              [0, 1,       dt],
              [0, 0,        1]])
Q_c = Matrix([[0, 0, 0],
              [0, 0, 0],
              [0, 0, 1]])*phi
Q = integrate(F_k * Q_c * F_k.T, (dt, 0, dt))
# factor phi out of the matrix to make it more readable
Q = Q / phi
MatMul(Q, phi)
print(Q)
#For Oth order
F_k = Matrix([[1]])
Q_c = Matrix([[phi]])
print('0th order discrete process noise')
integrate(F_k*Q_c*F_k.T,(dt, 0, dt))
Q = Q / phi
MatMul(Q, phi)
# Output = [∆tΦs]
# For 1st order
F_k = Matrix([[1, dt],
              [0, 1]])
Q_c = Matrix([[0, 0],
              [0, 1]]) * phi
Q = integrate(F_k * Q_c * F_k.T, (dt, 0, dt))
print('1st order discrete process noise')
Q = Q / phi
MatMul(Q, phi)
# Output = [∆t**3/3, ∆t**2/2; ∆t**2/2, ∆t]Φs

"""
# Piecewise White Noise Model
"""
#For 1st order
var = symbols('sigma^2_v')
v = Matrix([[dt**2 / 2], [dt]])
Q = v * var * v.T
# factor variance out of the matrix to make it more readable
Q = Q / var
MatMul(Q, var)
#For 2nd Order
var = symbols('sigma^2_v')
v = Matrix([[dt**2 / 2], [dt], [1]])
Q = v * var * v.T
# factor variance out of the matrix to make it more readable
Q = Q / var
MatMul(Q, var)

"""
Using FilterPy to Compute Q
"""

from filterpy.common import Q_continuous_white_noise
from filterpy.common import Q_discrete_white_noise
# For Continuous White Noise Model
Q = Q_continuous_white_noise(dim=2, dt=1, spectral_density=1)
print(Q)
# For Piecewise White Noise Model
Q = Q_discrete_white_noise(dim=2, var=1.)#1st order
print(Q)
Q = Q_discrete_white_noise(dim=3, var=1.)#2nd order
print(Q)

