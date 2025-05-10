#-*- coding: utf-8 -*-
"""
My Implementation
"""
import sympy as sp

def compute_discrete_process_noise(F_k, dt):
    t = sp.symbols('t')
    phi_acc = 1.0
    phi_yaw = 0.05
    phi_yawrate = 0.2
    #Q_c = diag([0, 0, 0, 0, phi_acc, phi_acc, phi_yaw, phi_yawrate])
    Q_c = sp.zeros(8, 8)
    Q_c[4, 4] = phi_acc
    Q_c[5, 5] = phi_acc
    Q_c[6, 6] = phi_yaw
    Q_c[7, 7] = phi_yawrate
    print(Q_c)
    print(Q_c.shape)
    Q = sp.integrate(F_k * Q_c * F_k.T, (t, 0, dt))
    print(Q)

def main():
    dt = 0.1
    Phi = sp.eye(8)
    Phi[0, 2] = dt           
    Phi[1, 3] = dt       
    Phi[0, 4] = 0.5 * dt**2 
    Phi[1, 5] = 0.5 * dt**2
    Phi[2, 4] = dt
    Phi[3, 5] = dt
    Phi[6, 7] = dt
    print(Phi)
    compute_discrete_process_noise(Phi, dt)
if __name__ == "__main__":
    main()

# from sympy import (init_printing, Matrix, MatMul,
# integrate, symbols)
# init_printing(use_latex='mathjax')
# dt, phi = symbols('\Delta{t} \Phi_s')
# F_k = Matrix([[1, dt, dt**2/2],
#               [0, 1,       dt],
#               [0, 0,        1]])
# Q_c = Matrix([[0, 0, 0],
#               [0, 0, 0],
#               [0, 0, 1]])*phi
# Q = integrate(F_k * Q_c * F_k.T, (dt, 0, dt))
# # factor phi out of the matrix to make it more readable
# Q = Q / phi
# print(MatMul(Q, phi))