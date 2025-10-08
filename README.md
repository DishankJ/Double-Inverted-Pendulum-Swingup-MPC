### Images for Pendulum up-down position stabilisation
![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_up-down_state_trajectory.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_up-down_control_trajectory.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_up-down.gif?raw=true)

### Images for Pendulum down-down position
![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_down-down_state_trajectory.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_down-down_control_trajectory.png?raw=true)

![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_down-down.gif?raw=true)

### Images for Pendulum down-up position
![alt](https://github.com/DishankJ/Double-Inverted-Pendulum-Swingup-MPC/blob/main/media/dip_down-up.gif?raw=true)

## Problem faced:
A major problem faced in this is that the time step taken is shorter than time taken for optimisation which varies but in general is still greater. Also, in the third swingup maneuver of down-up position it showed a poor state trajectory and optimisation time taken at each step even though the maneuver was finished at last.
