### Images for Pendulum up-up position stabilisation
In solving this problem an energy-based(KE - PE) objective function was used, which is also a reason why the cart didn't come back to '0', which can be done by constraining the x-coordinate of cart.

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_up_control_trajectory.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_up_state_trajectory.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/double_inv_pendulum_up__mpc.gif?raw=true)

### Images for Pendulum up-up position stabilisation
In solving this problem an energy-based(KE - PE) objective function was used, which is also a reason why the cart didn't come back to '0', which can be done by constraining the x-coordinate of cart.

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_down_state_trajectory.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/double_inv_pendulum_down_mpc.gif?raw=true)

## Problem faced:
1. Solving the MPC problem for the pendulum up case was very tricky since, in comparison to solving the up-up case, a better and appropriate cost function had to be chosen with carefully chosen weights as well.
