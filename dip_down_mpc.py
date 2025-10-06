# %%
import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

m0 = 1.5 
m1 = 0.5
m2 = 0.75
L1 = 0.5
L2 = 0.75
g = 9.8

l1 = L1/2
l2 = L2/2
J1 = (m1 * l1**2) / 3
J2 = (m2 * l2**2) / 3

def dynamics(x, u):
    x_dot = ca.SX.sym("x_dot", 6, 1)

    x_dot[0] = x[3]
    x_dot[1] = x[4]
    x_dot[2] = x[5]
    x_dot[3] = (4320.0*u[0]*np.cos(x[1] - x[2])**2 - 7040.0*u[0] - 540.0*(np.sin(x[1] - 2*x[2]) + np.sin(3*x[1] - 2*x[2]))*x[4]**2 + 303.75*(np.sin(2*x[1] - 3*x[2]) + np.sin(2*x[1] - x[2]))*x[5]**2 - 2160.0*np.sin(x[1] - x[2])*np.cos(x[1])*x[5]**2 + 1980.0*np.sin(x[1] - x[2])*np.cos(x[2])*x[4]**2 + 2160.0*np.sin(x[1])*np.cos(x[1] - x[2])**2*x[4]**2 - 42336.0*np.sin(x[1])*np.cos(x[1] - x[2])*np.cos(x[2]) - 3520.0*np.sin(x[1])*x[4]**2 + 37632.0*np.sin(2*x[1]) + 1215.0*np.sin(x[2])*np.cos(x[1] - x[2])**2*x[5]**2 - 42336.0*np.sin(x[2])*np.cos(x[1] - x[2])*np.cos(x[1]) - 1980.0*np.sin(x[2])*x[5]**2 + 19404.0*np.sin(2*x[2]))/(3780.0*np.cos(2*x[1] - 2*x[2]) + 1680.0*np.cos(2*x[1]) - 180.0*np.cos(2*x[2]) - 9760.0)
    x_dot[4] = (-0.285714285714286*u[0]*np.cos(x[1] - 2*x[2]) + 0.73015873015873*u[0]*np.cos(x[1]) - 4.9*np.sin(x[1] - 2*x[2]) + 0.5625*np.sin(x[1] - x[2])*x[5]**2 + 0.0625*np.sin(x[1] + x[2])*x[5]**2 + 0.25*np.sin(2*x[1] - 2*x[2])*x[4]**2 - 16.8777777777778*np.sin(x[1]) + 0.111111111111111*np.sin(2*x[1])*x[4]**2 ) / ( 0.25*np.cos(2*x[1] - 2*x[2]) + 0.111111111111111*np.cos(2*x[1]) - 0.0119047619047619*np.cos(2*x[2]) - 0.645502645502646 )
    x_dot[5] = (-960.0*u[0]*np.cos(2*x[1] - x[2]) + 800.0*u[0]*np.cos(x[2]) - 1500.0*np.sin(x[1] - x[2])*x[4]**2 - 40.0*np.sin(x[1] + x[2])*x[4]**2 - 472.5*np.sin(2*x[1] - 2*x[2])*x[5]**2 + 16464.0*np.sin(2*x[1] - x[2]) - 12152.0*np.sin(x[2]) - 22.5*np.sin(2*x[2])*x[5]**2)/(472.5*np.cos(2*x[1] - 2*x[2]) + 210.0*np.cos(2*x[1]) - 22.5*np.cos(2*x[2]) - 1220.0)

    return x_dot

# %%
NX = 6
NU = 1

N = 50
dt = 0.02
Nsim = 100
x_0 = np.array([[0], [np.pi], [np.pi], [0], [0], [0]])

x = ca.SX.sym('x', NX)
u = ca.SX.sym('u', NU)

x_dot = dynamics(x, u)

system = ca.Function('system', [x, u], [x_dot])

# %%
ode = {'x': x, 'ode': x_dot, 'p': u}
opts = {'tf': dt}

ode_solver = ca.integrator('F', 'idas', ode, opts)

lb_x = -30*np.ones((NX,1))
lb_x[0] = -6
lb_x[1] = -np.inf
lb_x[2] = -np.inf
lb_x.reshape((1, -1))

ub_x = 30*np.ones((NX,1))
ub_x[0] = 6
ub_x[1] = np.inf
ub_x[2] = np.inf
ub_x.reshape((1, -1))

lb_u = -60*np.ones((NU,1))
ub_u = 60*np.ones((NU,1))

# %%
# Q = np.diag([10,0.5,0.5,1,1,1])
# Qf = np.diag([100,20,20,1,1,1])
# R = 1
Q = np.diag([10,5,5,1,0.1,0.1])
Qf = np.diag([20,20,20,1,1,1])
R = 0.01

# w_stage_phi = 1
# w_term_phi = 10

# w_stage_energy = 1
# w_term_energy = 30


w_stage_energy = 1
w_term_energy = 100


# %%
E_kin_cart = 1 / 2 * m0 * x[3]**2
E_kin_p1 = 1 / 2 * m1 * ((x[3] + l1 * x[4] * ca.cos(x[1]))**2 + (l1 * x[4] * ca.sin(x[1]))**2) + 1 / 2 * J1 * x[4]**2
E_kin_p2 = 1 / 2 * m2 * ((x[3] + L1 * x[4] * ca.cos(x[1]) + l2 * x[5] * ca.cos(x[2]))**2 + (L1 * x[4] * ca.sin(x[1]) + \
                                                                                                    l2 * x[5] * ca.sin(x[2]))**2) + 1 / 2 * J2 * x[5]**2
E_kin = E_kin_cart + E_kin_p1 + E_kin_p2
E_pot = m1 * g * l1 * ca.cos(x[1]) + m2 * g * (L1 * ca.cos(x[1]) +
                                                l2 * ca.cos(x[2]))
E_target = m1*g*l1 + m2*g*(L1 + l2)

E_total = E_kin + E_pot
E_diff = E_kin - E_pot

x_target1 = np.array([[0], [0], [0], [0], [0], [0]])
x_target = ca.SX.sym("x_target", NX, 1)

x_diff = x - x_target


J_stage = x_diff.T@Q@x_diff + u.T@R@u
J_terminal = x_diff.T@Qf@x_diff

# J_phi1_phi2 = (ca.sin(0.5*x_diff[1])**2 + ca.sin(0.5*x_diff[2])**2)
# J_stage += w_stage_phi * J_phi1_phi2
# J_terminal += w_term_phi * J_phi1_phi2

# J_stage += w_stage_energy * (E_target - E_total)**2
# J_terminal += w_term_energy * (E_target - E_pot)**2 + w_term_energy * (E_kin)**2


J_stage += w_stage_energy * E_diff
J_terminal += w_term_energy * E_diff

stage_cost_fcn = ca.Function("J_stage",[x,u, x_target],[J_stage])
terminal_cost_fcn = ca.Function('J_terminal',[x, x_target],[J_terminal])

# %%
states = ca.SX.sym('states', NX, N+1)
controls = ca.SX.sym('controls', NU, N)

g = []
lb_g = []
ub_g = []

x_init = ca.SX.sym('x_init', NX)

g.append(states[:, 0] - x_init)
lb_g.append(np.zeros((NX,1)))
ub_g.append(np.zeros((NX,1)))

# %%
J = 0
for t in range(N):
    g.append(states[:, t+1] - (states[:, t] + dt*system(states[:, t], controls[:, t])))
    lb_g.append(np.zeros((NX,1)))
    ub_g.append(np.zeros((NX,1)))

    J += stage_cost_fcn(states[:, t], controls[:, t], x_target)
J += terminal_cost_fcn(states[:, N], x_target)

g = ca.vertcat(*g)
prob = {'f': J, 'x': ca.vertcat(ca.reshape(states, NX*(N+1), 1), ca.reshape(controls, NU*N, 1)), 'g':g, 'p': ca.vertcat(x_init, x_target)}
mpc_solver = ca.nlpsol('solver', 'ipopt', prob)

lb_g = ca.vertcat(*lb_g)
ub_g = ca.vertcat(*ub_g)

# %%
from casadi.tools import *

opt_x = struct_symSX([
    entry('states', shape=NX, repeat=[N+1]),
    entry('controls', shape=NU, repeat=[N])
])

lb_opt_x = opt_x(0)
ub_opt_x = opt_x(0)

# set desired bounds for input 
lb_opt_x['states'] = lb_x
ub_opt_x['states'] = ub_x

lb_opt_x['controls'] = lb_u
ub_opt_x['controls'] = ub_u

mpc_res = mpc_solver(p=ca.vertcat(x_0, x_target1), lbg=lb_g, ubg=ub_g, lbx = lb_opt_x, ubx = ub_opt_x)

# %%
opt_x_k = opt_x(mpc_res['x'])
X_k = np.array([*opt_x_k['states', :, 0]]).reshape(-1)
T1_k = np.array([*opt_x_k['states', :, 1]]).reshape(-1)
T2_k = np.array([*opt_x_k['states', :, 2]]).reshape(-1)

# plt.plot([dt*i for i in range(len(X_k))], X_k)
# plt.plot([dt*i for i in range(len(X_k))], T1_k)
# plt.plot([dt*i for i in range(len(X_k))], T2_k)

# %%
res_x_mpc = [x_0]
res_u_mpc = []
xsoll = x_target1

# %%
for i in range(Nsim):
    mpc_res = mpc_solver(p=vertcat(x_0, xsoll), lbg=lb_g, ubg=ub_g, lbx=lb_opt_x, ubx=ub_opt_x)

    if True:
        mpc_res = mpc_solver(p=vertcat(x_0, xsoll), x0=opt_x_k, lbg=lb_g, ubg=ub_g, lbx=lb_opt_x, ubx=ub_opt_x)
    
    opt_x_k = opt_x(mpc_res['x'])
    u_k = opt_x_k['controls', 0]

    res_integrator = ode_solver(x0=x_0, p=u_k)
    x_next = res_integrator['xf']
 
    x_0 = x_next

    res_x_mpc.append(x_next)
    res_u_mpc.append(u_k)
    # if i == 10:
    #     break
    
    print('------------------------------------------------------------------------------ ITER', i, '------------------------------------------------------------------------------')
    
res_x_mpc = np.concatenate(res_x_mpc, axis=1)
res_u_mpc = np.concatenate(res_u_mpc, axis=1)

# %%
plt.plot([dt*i for i in range(len(res_x_mpc[0,:]))], res_x_mpc[0,:])
plt.plot([dt*i for i in range(len(res_x_mpc[1,:]))], res_x_mpc[1,:])
plt.plot([dt*i for i in range(len(res_x_mpc[2,:]))], res_x_mpc[2,:])
plt.grid()
plt.legend(['x', 'theta1', 'theta2'])
plt.show()

# %%
# animation
fig, ax = plt.subplots()
ax.set_xlim(-4, 6)
ax.set_ylim(-1.5, 1.5)
plt.grid()

x1_mass = res_x_mpc[0,:]
x1_pendulum = res_x_mpc[0,:] + L1 * np.sin(res_x_mpc[1,:])
y1_pendulum = L1 * np.cos(res_x_mpc[1,:])
x2_pendulum = res_x_mpc[0,:] + L1 * np.sin(res_x_mpc[1,:]) + L2 * np.sin(res_x_mpc[2,:])
y2_pendulum = L1 * np.cos(res_x_mpc[1,:]) + L2 * np.cos(res_x_mpc[2,:])

def update(frame):
    pendulum1.set_data([x1_mass[frame], x1_pendulum[frame]], [0, y1_pendulum[frame]])
    pendulum2.set_data([x1_pendulum[frame], x2_pendulum[frame]], [y1_pendulum[frame], y2_pendulum[frame]])
    mass1.set_data([x1_mass[frame]], [0])
    mass2.set_data([x1_pendulum[frame]], [y1_pendulum[frame]])
    mass3.set_data([x2_pendulum[frame]], [y2_pendulum[frame]])
    return mass1, pendulum1, mass2, pendulum2, mass3

mass1, = ax.plot([x1_mass[0]], [0], 'o', markersize=4*int(m0)+1, color='red')
pendulum1, = ax.plot([x1_mass[0], x1_pendulum[0]], [0, y1_pendulum[0]], lw=2, color='blue')
mass2, = ax.plot([x1_pendulum[0]], [y1_pendulum[0]], 'o', markersize=4*int(m1)+1, color='red')
pendulum2, = ax.plot([x1_pendulum[0], x2_pendulum[0]], [y1_pendulum[0], y2_pendulum[0]], lw=2, color='orange')
mass3, = ax.plot([x2_pendulum[0]], [y2_pendulum[0]], 'o', markersize=4*int(m2)+1, color='red')

animation = FuncAnimation(fig, update, frames=len(res_x_mpc[0,:]), interval=int(1/dt), blit=True)
animation.save('double_inv_pendulum_mpc.gif', writer='pillow')
plt.show()
