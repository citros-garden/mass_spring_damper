# General Info

This is a simple ROS demonstration of a mass-spring-damper system.
In the ROS system we have two nodes: the first represents the dynamics and the second one is the controller.

![](https://user-images.githubusercontent.com/58637596/194471956-3d61c70f-3d68-4e00-90c7-8a64c35b29aa.png)

# System dynamics

With Newhon's second law the equations of motion:

![](https://user-images.githubusercontent.com/58637596/194473030-bce2ac10-046a-4077-a72c-df59c4bc1300.jpg)

and we get a simple second order system:

![](https://user-images.githubusercontent.com/58637596/194473493-a8976715-7299-4295-bf59-37bad417c7be.jpg)

where the natural frequency &omega;<sub>n</sub> =  <span>&#8730;</span>k/m

You can choose the system's parameters m, k and c and choose the initial condition x0, v0 and a0 and set them in the [dynamics parameters file](https://github.com/lulav/demo_lulav_elbit/blob/foxy/src/dynamics/config/params.yaml)


# The controller

You can write your own controller to try stabilize the system for a given setpoint.

the default controller is a simple PID controller with the following form:

F(t) = k<sub>p</sub>*e(t) + k<sub>i</sub><span>&#8747;</span>e(t)dt + k<sub>d</sub>d&frasl;<sub>dt</sub>(e(t))

you can tune the controller gains on the [parameters file](https://github.com/lulav/demo_lulav_elbit/blob/foxy/src/controller/config/params.yaml)


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


to the the default controller:

                ros2 launch dynamics dynamics_controller.launch


![](https://user-images.githubusercontent.com/58637596/194480849-513b2441-8c73-480e-888e-95976b35b263.gif)


