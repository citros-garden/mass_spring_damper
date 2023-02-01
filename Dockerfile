FROM ros:foxy

ENV ROS_DISTRO foxy

# install ros package
RUN apt-get update && apt-get install -y \
    python3-pip \
    curl \
      # ros-${ROS_DISTRO}-demo-nodes-cpp \
      # ros-${ROS_DISTRO}-demo-nodes-py \    
    && rm -rf /var/lib/apt/lists/* 


WORKDIR /app
COPY . .
RUN colcon build

# CITROS
# RUN apt-get install python-pip
RUN sudo apt update && sudo apt install -y ros-foxy-rosbag2-storage-mcap
# RUN sudo apt update & sudo apt install ros-rolling-rosbag2-storage-mcap
RUN pip install utils/citros_cli
# RUN pip install LRS-Lulav
# END CITROS





RUN chmod +x ros2_entrypoint.sh
ENTRYPOINT ["/app/ros2_entrypoint.sh"]

CMD ["bash"]
