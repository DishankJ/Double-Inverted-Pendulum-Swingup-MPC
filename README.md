### Images for Pendulum up-up position stabilisation
In solving this problem an energy-based(KE - PE) objective function was used, which is also a reason why the cart didn't come back to '0', which can be done by constraining the x-coordinate of cart.

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_up_control_trajectory.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_up_state_trajectory.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/double_inv_pendulum_up__mpc.gif?raw=true)

## Limitations of this solution:
1. Towards the end of the simulation, IPOPT linear solver, mumps solver was taking longer time ~0.6s than the time step which is very high considering the discretization time step was taken to be 0.05s.
