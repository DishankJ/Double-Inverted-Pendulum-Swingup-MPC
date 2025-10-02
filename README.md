# Double-Inverted-Pendulum-Swingup-MPC

### Images for Pendulum down-down position stabilisation
Here, a standard quadratic cost function was used in which Q, Qf and R - the weighting matrices - were positive definite.

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_down_control_trajectory.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_down_state_trajectories.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/double_inv_pendulum_down_mpc.gif?raw=true)


### Images for Pendulum up-up position stabilisation
In solving this problem an energy-based(KE - PE) objective function was used, which is also a reason why the cart didn't come back to '0', which can be done by constraining the x-coordinate of cart.

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_up_control_trajectory.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_up_state_trajectories.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/double_inv_pendulum_up_mpc.gif?raw=true)

## Limitations of this solution:
1. Towards the end of the simulation, IPOPT linear solver, mumps solver was taking ~1.5seconds which is very high considering the discretization time step was taken to be 0.05s.
2. Increasing the weight on control lead to oscillations in theta in the case of pendulum down-down position which is intuitively clear however in a similar situation [here](https://www.do-mpc.com/en/v4.1.0/example_gallery/DIP.html), they could achieve that with a bounded input of 4 Newton.
