from filterpy.common import Q_continuous_white_noise
from filterpy.common import Q_discrete_white_noise
import sympy
from sympy import (init_printing, Matrix, MatMul, 
                   integrate, symbols)
from scipy.linalg import block_diag

Q = Q_continuous_white_noise(dim=3, dt=0.05, spectral_density=1)
# print(Q)

Q = Q_discrete_white_noise(3, var=1.)
# print(Q)

var = symbols('sigma^2_v')
dt = symbols('dt')
v = Matrix([[dt**2 / 2], [dt], [1]])

Q = v * var * v.T

# factor variance out of the matrix to make it more readable
Q = Q / var
print(MatMul(Q, var))



q = Q_discrete_white_noise(dim=2, dt=dt, var=0.001)
Q = block_diag(q, q)
# print(q)
# print(Q)