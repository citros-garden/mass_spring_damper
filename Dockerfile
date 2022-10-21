FROM ros:foxy

ENV ROS_DISTRO foxy

# install ros package
RUN apt-get update && apt-get install -y \
    python3-pip \
    && apt-get install curl \
      # ros-${ROS_DISTRO}-demo-nodes-cpp \
      # ros-${ROS_DISTRO}-demo-nodes-py \    
    && rm -rf /var/lib/apt/lists/* 
# RUN apt-get install python-pip

WORKDIR /app
COPY . .
RUN colcon build
RUN pip install utils/LRS

RUN chmod +x ros2_entrypoint.sh
ENTRYPOINT ["/app/ros2_entrypoint.sh"]

CMD ["bash"]
