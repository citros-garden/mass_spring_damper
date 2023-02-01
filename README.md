# General Info

This is a simple ROS demonstration of a mass-spring-damper system.
In the ROS system we have two nodes: the first represents the dynamics and the second one is the controller.

![](https://user-images.githubusercontent.com/58637596/194718349-455a479d-434e-45a6-86bc-0a4f5d95dd49.png)

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

$$f(t) = {k_pe(t) + k_i\int{e(t)dt}} + k_d {d\over dt}(e(t))$$

you can tune the controller gains, $k_p$, $k_i$, $k_d$ on the [controller parameters file](https://github.com/lulav/demo_lulav_elbit/blob/foxy/src/controller/config/params.yaml)


# Installation

clone the repository to your local machine:
                
                git clone git@github.com:lulav/demo_lulav_elbit.git

open the repository in the VScode:

                cd ~/demo_lulav_elbit
                code .

open the repository in the container from VScode with `reopen in container` option.

build and source the workspace:

                colcon build
                source install/local_setup.bash

# Run the default example:

you can run the free-system without controller with this launch file:

                ros2 launch dynamics dynamics.launch.py


to run the default controller:

                ros2 launch dynamics dynamics_controller.launch.py

<video src='https://user-images.githubusercontent.com/58637596/194520348-c97344c0-b9be-4ad5-ba11-29188c18011e.mp4' width=600/>


## Docker build

        citros docker build -t vovacooper/demo_lulav_elbit:latest .

## Docker run 

        citros docker run --rm -it --net=host vovacooper/demo_lulav_elbit:latest citros run beebcb55-6110-4be4-8fec-05af808ce6fc 1
        
        
        docker run --rm -it --net=host vovacooper/demo_lulav_elbit:latest ros2 launch launches/LRS.launch.py user_id:=63502ab7865fb52ab569e90c project_id:=6351584232818a188f45fd59 simulation_id:=63523680846b4ecaf0404d00 simulation_instance_id:=635236a7846b4ecaf0404d01 simulation_instance_seq:=0


        docker run --rm -it --net=host -e "CITROS_ENTRYPOINT"="http://host.docker.internal/api/graphql" \
        -e "CITROS_LOGS"="http://host.docker.internal/logs" \
        -e "CITROS_BAG"="http://host.docker.internal/bag" \
        vovacooper/demo_lulav_elbit:latest \
        citros run 2949dfbb-2545-479e-a42c-de1ce038f5c5 1 \
        --key eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJyb2xlIjoiY2l0cm9zX2FkbWluIiwidXNlcl9pZCI6IjNjYWEyMjNhLWFhNzQtNDFlZS05MmEyLTViZWUzOTkyMzg1OSIsInVzZXJfbmFtZSI6Im5vYW1vb24iLCJjaXRyb3Nfcm9sZSI6InVzZXIiLCJvcmdhbml6YXRpb25faWQiOiJlOTE1ZDYzOS02MzcyLTQ1ZTQtODU1ZC1hOGM5YjdkNmFiMDIiLCJvcmdhbml6YXRpb25fdHlwZSI6Ik1BTkFHRSIsImRvbWFpbl9wcmVmaXgiOiJsdWxhdiIsImV4cCI6MTY3NTMyNDcxMSwiaWF0IjoxNjc1MjM4MzExLCJhdWQiOiJwb3N0Z3JhcGhpbGUiLCJpc3MiOiJwb3N0Z3JhcGhpbGUifQ.YJvZfK5jKx7bLStCOhmOQxP5yo7K6MygFI-Te5z5BW0 
        
        





