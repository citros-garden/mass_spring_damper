# FROM althack/ros2:humble-dev 
FROM althack/ros2:humble-full 

# Set up auto-source of workspace for ros user
RUN pip install setuptools==58.2.0
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
