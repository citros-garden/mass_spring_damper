FROM ros:foxy

ENV ROS_DISTRO foxy

# install ros package
RUN apt-get update && apt-get install -y \
    && apt-get install curl \
      # ros-${ROS_DISTRO}-demo-nodes-cpp \
      # ros-${ROS_DISTRO}-demo-nodes-py \    
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . .
RUN colcon build

ENTRYPOINT ["ros2_entrypoint.sh"]

CMD ["bash"]
