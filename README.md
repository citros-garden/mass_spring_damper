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

You can choose the system's parameters `m`, `k` and `c` and choose the initial condition `x0`, `v0` and `a0` and set them in the [dynamics parameters file](src/dynamics/config/params.yaml)


# The controller

You can write your own controller to try stabilize the system for a given setpoint.

the default controller is a simple PID controller with the following form:

$$f(t) = {k_pe(t) + k_i\int{e(t)dt}} + k_d {d\over dt}(e(t))$$

you can tune the controller gains, $k_p$, $k_i$, $k_d$ on the [controller parameters file](src/controller/config/params.yaml)


# Installation

clone the repository to your local machine:
                
                git clone git@github.com:citros-garden/mass_spring_damper.git

open the repository in the VScode:

                cd ~/mass_spring_damper
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



# CITROS
```bash 

## Docker build
citros docker build --no-cache -t mass_spring_dumper .

## Docker run 
citros docker run --rm -it --net=host mass_spring_dumper
citros docker run --rm -it --net=host mass_spring_dumper citros run beebcb55-6110-4be4-8fec-05af808ce6fc 1

# run from local machine.
citros docker run --rm -it --net=host -e "CITROS_ENTRYPOINT"="http://host.docker.internal/api/graphql" \
-e "CITROS_LOGS"="http://host.docker.internal/logs" \
-e "CITROS_BAG"="http://host.docker.internal/bag" \
-e "CITROS_DATA_DATABASE"="lulav" \
-e "CITROS_DOMAIN"="http://host.docker.internal" \
-e "CITROS_DATA_HOST"="host.docker.internal" \
mass_spring_dumper:latest \
citros run 02ecc4c5-6680-46aa-83c1-67c93a172b9e 0 \
--key eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJyb2xlIjoiY2l0cm9zX2FkbWluIiwidXNlcl9pZCI6IjgwOGI4OGM2LTQ1YWItNDgxMS1iZGNjLTRhZmNlNTkxZjg0NSIsInVzZXJfbmFtZSI6InZvdmFjb29wZXIiLCJjaXRyb3Nfcm9sZSI6InVzZXIiLCJvcmdhbml6YXRpb25faWQiOiJlOTE1ZDYzOS02MzcyLTQ1ZTQtODU1ZC1hOGM5YjdkNmFiMDIiLCJvcmdhbml6YXRpb25fdHlwZSI6Ik1BTkFHRSIsImRvbWFpbl9wcmVmaXgiOiJsdWxhdiIsImV4cCI6MTY3NzQyODk4NCwiaWF0IjoxNjc3MzQyNTg0LCJhdWQiOiJwb3N0Z3JhcGhpbGUiLCJpc3MiOiJwb3N0Z3JhcGhpbGUifQ.RUYY5VT_oSzcRaht-xk7SOUOHoD4ykCyrcaUQ5sLUXk

```



## gcloud: docker

https://console.cloud.google.com/artifacts/browse/citros?project=citros&supportedpurview=project

```bash
# if building from linux machine
docker build -t mass_spring_dumper . 
# *** when building from MAC M1 chip add FROM --platform=linux/amd64 ***
docker buildx build --platform linux/amd64 -t mass_spring_dumper .   

# login to citros
citros login
# login with docker
citros docker-login

# upload to local registry
docker tag mass_spring_dumper localhost:5001/citros/lulav/mass_spring_dumper
docker push localhost:5001/citros/lulav/mass_spring_dumper
# upload to google artifact registry
docker tag mass_spring_dumper us-central1-docker.pkg.dev/citros/lulav/mass_spring_dumper:latest
docker push us-central1-docker.pkg.dev/citros/lulav/mass_spring_dumper:latest

```


Jfrog
```bash

# tag  
docker tag mass_spring_dumper citros.jfrog.io/dev-virtual-docker/citros/mass_spring_dumper:0.0.1
# push
docker push citros.jfrog.io/dev-virtual-docker/citros/mass_spring_dumper:0.0.1

# run
citros docker run --rm -it --net=host -e "CITROS_ENTRYPOINT"="https://citros.io/api/graphql" \
-e "CITROS_LOGS"="https://citros.io/logs" \
-e "CITROS_BAG"="https://citros.io/bag" \
mass_spring_dumper:latest \
citros run a9ff71c7-69c5-4b57-905c-195c5b4753bc 1 \
--key eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJyb2xlIjoiY2l0cm9zX2FkbWluIiwidXNlcl9pZCI6IjNjYWEyMjNhLWFhNzQtNDFlZS05MmEyLTViZWUzOTkyMzg1OSIsInVzZXJfbmFtZSI6Im5vYW1vb24iLCJjaXRyb3Nfcm9sZSI6InVzZXIiLCJvcmdhbml6YXRpb25faWQiOiJlOTE1ZDYzOS02MzcyLTQ1ZTQtODU1ZC1hOGM5YjdkNmFiMDIiLCJvcmdhbml6YXRpb25fdHlwZSI6Ik1BTkFHRSIsImRvbWFpbl9wcmVmaXgiOiJsdWxhdiIsImV4cCI6MTY3NTQzODc0MSwiaWF0IjoxNjc1MzUyMzQxLCJhdWQiOiJwb3N0Z3JhcGhpbGUiLCJpc3MiOiJwb3N0Z3JhcGhpbGUifQ.ypzeTPBAUW433OlMqNk1Piq7lqR6MwpxlZqEYytloJ4

```

