### Images for Pendulum up-up position stabilisation
![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_up_control_trajectory.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_up_state_trajectory.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/double_inv_pendulum_up__mpc.gif?raw=true)

### Images for Pendulum up-up position stabilisation
Cost function used here was taken from existing solutions since a simple energy minimization doesn't work possibly because MPC doesn't know if the problem is feasible or look that far ahead into the future to solve it.

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_down_state_trajectory.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/double_inv_pendulum_down_mpc.gif?raw=true)

## Problem faced:
1. Solving the MPC problem for the pendulum up case was very tricky since, in comparison to solving the up-up case, a better and appropriate cost function had to be chosen with carefully chosen weights as well.
