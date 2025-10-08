import sympy as sym
from sympy import Matrix, symbols, euler_equations
from sympy.physics.vector import dynamicsymbols
import numpy as np
import casadi as ca

u = symbols('u')
x = dynamicsymbols('x')
theta1 = dynamicsymbols('theta1')
theta2 = dynamicsymbols('theta2')
dx = dynamicsymbols('x', 1)
dtheta1 = dynamicsymbols('theta1', 1)
dtheta2 = dynamicsymbols('theta2', 1)
ddx = dynamicsymbols('x', 2)
ddtheta1 = dynamicsymbols('theta1', 2)
ddtheta2 = dynamicsymbols('theta2', 2)

# original config
# m0 = 1.5 
# m1 = 0.5
# m2 = 0.75
# L1 = 0.5
# L2 = 0.75

# new config
m0 = 0.6 
m1 = 0.2
m2 = 0.2
L1 = 0.5
L2 = 0.5

l1 = L1/2
l2 = L2/2
g = 9.8

I1 = (1/12)*m1*L1**2
I2 = (1/12)*m2*L2**2

coords_m0 = Matrix([[x],
                    [0]])
coords_m1 = Matrix([[x + L1/2*sym.sin(theta1)],
                    [L1/2*sym.cos(theta1)]])
coords_m2 = Matrix([[x + L1*sym.sin(theta1) + L2/2*sym.sin(theta2)],
                    [L1*sym.cos(theta1) + L2/2*sym.cos(theta2)]])

v0 = sym.diff(coords_m0, 't')
v1 = sym.diff(coords_m1, 't')
v2 = sym.diff(coords_m2, 't')

E_kin0 = (1/2)*m0*(v0.T*v0)[0]
E_kin1 = (1/2)*m1*(v1.T*v1)[0] + (1/2)*I1*dtheta1**2
E_kin2 = (1/2)*m2*(v2.T*v2)[0] + (1/2)*I2*dtheta2**2

E_pot = m1*g*coords_m1[1] + m2*g*coords_m2[1]

L = E_kin0 + E_kin1 + E_kin2 - E_pot

eqs = euler_equations(L, [x, theta1, theta2], 't')
eq1 = sym.Eq(eqs[0].lhs + u, 0).simplify()
eq2 = eqs[1].simplify()
eq3 = eqs[2].simplify()

solutions = sym.solve([eq1, eq2, eq3], (ddx, ddtheta1, ddtheta2))
states_dot = sym.Matrix([[dx], [dtheta1], [dtheta2], [solutions[ddx]], [solutions[ddtheta1]], [solutions[ddtheta2]]])

func = sym.lambdify((x, theta1, theta2, dx, dtheta1, dtheta2, u), states_dot, modules='numpy')

def get_dynamics(x, u):
    x_dot = ca.SX.sym("x_dot", 6, 1)
    x_dot = func(x[0], x[1], x[2], x[3], x[4], x[5], u[0])
    return x_dot