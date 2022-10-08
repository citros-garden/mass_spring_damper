# General Info

This is a simple ROS demonstration of a mass-spring-damper system.
In the ROS system we have two nodes: the first represents the dynamics and the second one is the controller.

![](https://user-images.githubusercontent.com/58637596/194471956-3d61c70f-3d68-4e00-90c7-8a64c35b29aa.png)

# System dynamics

The system's equations of motion:

$$m\ddot x =  kf(t) -c\dot x -kx$$

and after laplace transformation (with zero I.C) we get a second order system:

$${X \over F} = {\omega_n^2 \over s^2 +2\omega_n\zeta s + \omega_n^2} $$

where the natural frequency $\omega_n = \sqrt{k \over m}$

You can choose the system's parameters `m`, `k` and `c` and choose the initial condition `x0`, `v0` and `a0` and set them in the [dynamics parameters file](https://github.com/lulav/demo_lulav_elbit/blob/foxy/src/dynamics/config/params.yaml)


# The controller

You can write your own controller to try stabilize the system for a given setpoint.

the default controller is a simple PID controller with the following form:

$$F(t) = {k_pe(t) + k_i\int{e(t)dt}} + k_d {d\over dt}(e(t))$$

you can tune the controller gains, $k_p$, $k_i$, $k_d$ on the [controller parameters file](https://github.com/lulav/demo_lulav_elbit/blob/foxy/src/controller/config/params.yaml)


# Installation

clone the repository to your local machine:
                
                git clone git@github.com:lulav/demo_lulav_elbit.git

open the repository in the VScode:

                cd ~/demo_lulav_elbit
                code .

open the repository in the container from VScode

# Run the default example:

you can run the free-system without controller with this launch file:

                ros2 launch dynamics dynamics.launch


to run the default controller:

                ros2 launch dynamics dynamics_controller.launch


<video src='https://user-images.githubusercontent.com/58637596/194520348-c97344c0-b9be-4ad5-ba11-29188c18011e.mp4' width=600/>


