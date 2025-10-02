import casadi as ca
import numpy as np
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

N = 50 # horizon length
dt = 0.05 # time step
# N = 20
# dt = 0.2

Q = ca.diag([10,2,2,0.5,0.5,0.5])
Qf = ca.diag([100,2,2,0.5,0.5,0.5])
R = ca.diag([0.1])


opti = ca.Opti()

def calculate_energy(x, t):
    E_kin_cart = 1 / 2 * m0 * x[3,t]**2
    E_kin_p1 = 1 / 2 * m1 * ((x[3,t] + l1 * x[4,t] * ca.cos(x[1,t]))**2 + (l1 * x[4,t] * ca.sin(x[1,t]))**2) + 1 / 2 * J1 * x[4,t]**2
    E_kin_p2 = 1 / 2 * m2 * ((x[3,t] + L1 * x[4,t] * ca.cos(x[1,t]) + l2 * x[5,t] * ca.cos(x[2,t]))**2 + (L1 * x[4,t] * ca.sin(x[1,t]) + \
                                                                                                        l2 * x[5,t] * ca.sin(x[2,t]))**2) + 1 / 2 * J2 * x[5,t]**2
    E_kin = E_kin_cart + E_kin_p1 + E_kin_p2
    E_pot = m1 * g * l1 * ca.cos(x[1,t]) + m2 * g * (L1 * ca.cos(x[1,t]) +
                                                    l2 * ca.cos(x[2,t]))
    cost = E_kin - E_pot
    return cost

def solve_mpc(x0):
    x = opti.variable(6, N+1)
    u = opti.variable(1, N)

    cost = 0.0

    # cost forumlation to mminimize kinetic energy and maximize potential energy
    for t in range(N):
        cost += calculate_energy(x, t)
        cost += 0.5*u[:, t].T@R@u[:, t]
    cost += calculate_energy(x, N)

    # alternative cost formulation to stabilize around the upright position
    # for t in range(N):
    #     cost += 0.5*x[:, t].T@Q@x[:, t] # stage cost
    #     cost += 0.5*u[:, t].T@R@u[:, t]
    # cost += 0.5*x[:, N].T@Qf@x[:, N] # terminal cost

    opti.minimize(cost)

    # system dynamics linearized around the upright(up-up) position
    A = ca.DM([
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [0.0, -7.35, 0.7875, 0.0, 0.0, 0.0],
        [0.0, 73.5, -33.075, 0.0, 0.0, 0.0],
        [0.0, -58.8, 51.1, 0.0, 0.0, 0.0]
    ])
    B = ca.DM([
        [0.0],
        [0.0],
        [0.0],
        [0.60714],
        [-1.5],
        [0.2857]
    ])

    # system dynamics linearized around the down-down position
    # A = ca.DM([
    #     [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
    #     [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    #     [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    #     [0.0, -7.35, 0.7875, 0.0, 0.0, 0.0],
    #     [0.0, -73.5, 33.075, 0.0, 0.0, 0.0],
    #     [0.0, 58.8, -51.1, 0.0, 0.0, 0.0]
    # ])
    # B = ca.DM([
    #     [0.0],
    #     [0.0],
    #     [0.0],
    #     [0.60714],
    #     [1.5],
    #     [-0.2857]
    # ])

    # problem constraints
    opti.subject_to(x[:, 0] == x0)
    for t in range(N):
        opti.subject_to(x[:, t+1] == x[:, t] + dt*A@x[:, t] + dt*B@u[:, t]) # dynamics constraint
        # opti.subject_to(x[:, t+1] == x[:, t] + dt*A@(x[:, t] - ca.DM([[0], [np.pi], [np.pi], [0], [0], [0]])) + dt*B@u[:, t]) # dynamics constraint
    opti.subject_to(opti.bounded(-100, u, 100))
    # opti.subject_to(opti.bounded(-8, x, 8))

    opti.solver('ipopt')
    sol = opti.solve()

    return sol.value(x), sol.value(u)

if __name__ == "__main__":
    x0 = opti.parameter(6)
    x0_np = np.array([0.0, 0.4, 0.2, 0.0, 0.0, 0.0])
    # x0_np = np.array([0.0, 0.99*np.pi, 0.99*np.pi, 0.0, 0.0, 0.0])
    opti.set_value(x0, x0_np)

    time_length = 100
    states_buffer = {'x': [x0_np[0]], 'theta1': [x0_np[1]], 'theta2': [x0_np[2]]}
    control_buffer = [0.0]
    time_gone_by = [0.0]
    for i in range(time_length):
        x_sol, u_sol = solve_mpc(x0)
        x0 = x_sol[:, 1]
        states_buffer['x'].append(float(x0[0]))
        states_buffer['theta1'].append(float(x0[1]))
        states_buffer['theta2'].append(float(x0[2]))
        control_buffer.append(float(u_sol[0]))
        time_gone_by.append((i+1)*dt)
    
    # plots
    plt.plot(time_gone_by, states_buffer['x'])
    plt.plot(time_gone_by, states_buffer['theta1'])
    plt.plot(time_gone_by, states_buffer['theta2'])
    plt.xlabel('time (s)')
    plt.ylabel('states')
    plt.legend(['x(m)', 'theta1(rad)', 'theta2(rad)'])
    plt.title('State trajectories')
    plt.show()
    plt.plot(time_gone_by, control_buffer)
    plt.xlabel('time (s)')
    plt.ylabel('u (N)')
    plt.title('Control')
    plt.show()

    # animation
    fig, ax = plt.subplots()
    ax.set_xlim(-4, 6)
    ax.set_ylim(-1.5, 1.5)
    plt.grid()

    x1_mass = states_buffer['x']
    x1_pendulum = states_buffer['x'] + L1 * np.sin(states_buffer['theta1'])
    y1_pendulum = L1 * np.cos(states_buffer['theta1'])
    x2_pendulum = states_buffer['x'] + L1 * np.sin(states_buffer['theta1']) + L2 * np.sin(states_buffer['theta2'])
    y2_pendulum = L1 * np.cos(states_buffer['theta1']) + L2 * np.cos(states_buffer['theta2'])

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

    animation = FuncAnimation(fig, update, frames=len(time_gone_by), interval=int(1/dt), blit=True)
    animation.save('double_inv_pendulum_mpc.gif', writer='pillow')
    plt.show()