FROM ros:humble

ENV ROS_DISTRO humble

# install citros
RUN apt update && apt install -y python3-pip
RUN pip install citros
RUN pip install citros --upgrade

# copy and build workspace
WORKDIR /app
COPY src src

RUN colcon build
RUN echo "source /app/install/setup.bash" >> ~/.bashrc

COPY ros2_entrypoint.sh ros2_entrypoint.sh
RUN chmod +x ros2_entrypoint.sh
ENTRYPOINT ["/app/ros2_entrypoint.sh"]

CMD ["bash"]
