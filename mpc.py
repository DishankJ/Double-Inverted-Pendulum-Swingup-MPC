import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from dynamics import m0, m1, m2, L1, L2, l1, l2, I1, I2, g, get_dynamics

NX = 6
NU = 1

N = 50
dt = 0.02
Nsim = 160
x_0 = np.array([[0], [0], [0.99*np.pi], [0], [0], [0]])

x = ca.SX.sym('x', NX)
u = ca.SX.sym('u', NU)

x_dot = get_dynamics(x, u)

system = ca.Function('system', [x, u], [x_dot])


ode = {'x': x, 'ode': x_dot, 'p': u}
opts = {'tf': dt}

ode_solver = ca.integrator('F', 'idas', ode, opts)

# bounds
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

lb_u = -20*np.ones((NU,1))
ub_u = 20*np.ones((NU,1))

# weighting matrices

# for original config, control bound was 60N
# Q = np.diag([10,5,5,1,0.1,0.1])
# Qf = np.diag([20,20,20,1,1,1])
# R = 0.01
# w_stage_energy = 1
# w_term_energy = 100

# for new config of system, control bounds are kept to 20N
Q = np.diag([0.5,2,2,0.01,0.01,0.01])
Qf = np.diag([80,5,10,0.5,1,1])
R = 0.1
w_stage_energy = 1
w_term_energy = 20


# costs
E_kin_cart = 1 / 2 * m0 * x[3]**2
E_kin_p1 = 1 / 2 * m1 * ((x[3] + l1 * x[4] * ca.cos(x[1]))**2 + (l1 * x[4] * ca.sin(x[1]))**2) + 1 / 2 * I1 * x[4]**2
E_kin_p2 = 1 / 2 * m2 * ((x[3] + L1 * x[4] * ca.cos(x[1]) + l2 * x[5] * ca.cos(x[2]))**2 + (L1 * x[4] * ca.sin(x[1]) + \
                                                                                                    l2 * x[5] * ca.sin(x[2]))**2) + 1 / 2 * I2 * x[5]**2
E_kin = E_kin_cart + E_kin_p1 + E_kin_p2
E_pot = m1 * g * l1 * ca.cos(x[1]) + m2 * g * (L1 * ca.cos(x[1]) + l2 * ca.cos(x[2]))
E_target = m1*g*l1 + m2*g*(L1 + l2)

E_total = E_kin + E_pot
E_diff = E_kin - E_pot

x_target1 = np.array([[0], [0], [0], [0], [0], [0]])
x_target = ca.SX.sym("x_target", NX, 1)

x_diff = x - x_target


J_stage = x_diff.T@Q@x_diff + u.T@R@u
J_terminal = x_diff.T@Qf@x_diff

J_stage += w_stage_energy * E_diff
J_terminal += w_term_energy * E_diff

stage_cost_fcn = ca.Function("J_stage",[x,u, x_target],[J_stage])
terminal_cost_fcn = ca.Function('J_terminal',[x, x_target],[J_terminal])


states = ca.SX.sym('states', NX, N+1)
controls = ca.SX.sym('controls', NU, N)

g = []
lb_g = []
ub_g = []

x_init = ca.SX.sym('x_init', NX)

g.append(states[:, 0] - x_init)
lb_g.append(np.zeros((NX,1)))
ub_g.append(np.zeros((NX,1)))


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


opt_x_k = opt_x(mpc_res['x'])
X_k = np.array([*opt_x_k['states', :, 0]]).reshape(-1)
T1_k = np.array([*opt_x_k['states', :, 1]]).reshape(-1)
T2_k = np.array([*opt_x_k['states', :, 2]]).reshape(-1)

res_x_mpc = [x_0]
res_u_mpc = []
xsoll = x_target1

# mpc loop
for i in range(Nsim):
    print('------------------------------------- ITERATION', i+1, '-------------------------------------')
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


#plots   
res_x_mpc = np.concatenate(res_x_mpc, axis=1)
res_u_mpc = np.concatenate(res_u_mpc, axis=1)

plt.plot([dt*i for i in range(len(res_x_mpc[0,:]))], res_x_mpc[0,:])
plt.plot([dt*i for i in range(len(res_x_mpc[1,:]))], res_x_mpc[1,:])
plt.plot([dt*i for i in range(len(res_x_mpc[2,:]))], res_x_mpc[2,:])
plt.grid()
plt.legend(['x (m)', 'theta1 (rad)', 'theta2 (rad)'])
plt.show()
plt.plot([dt*i for i in range(len(res_u_mpc[0,:]))], res_u_mpc[0,:])
plt.grid()
plt.legend(['u (in N)'])
plt.show()

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
animation.save('media/dip_up-down.gif', writer='pillow')
plt.show()
