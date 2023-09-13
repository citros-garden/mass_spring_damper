FROM ros:humble

ENV ROS_DISTRO humble

WORKDIR /app
COPY src src
RUN colcon build
RUN echo "source /app/install/setup.bash" >> ~/.bashrc

RUN pip install citros
RUN pip install citros --upgrade

RUN chmod +x ros2_entrypoint.sh
ENTRYPOINT ["/app/ros2_entrypoint.sh"]

CMD ["bash"]
