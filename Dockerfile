FROM ros:humble

ENV ROS_DISTRO humble

# install ros package
RUN apt-get update && apt-get install -y \
    python3-pip \
    curl \      
    && rm -rf /var/lib/apt/lists/* 

RUN sudo apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    ros-$ROS_DISTRO-rosbag2 \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-ros2bag \
    ros-$ROS_DISTRO-rosbag2-transport \
    && rm -rf /var/lib/apt/lists/* 


WORKDIR /app
COPY . .
RUN colcon build


# CITROS
# RUN apt-get install python-pip
# RUN sudo apt update && sudo apt install -y ros-humble-rosbag2-storage-mcap ros-humble-rosbag2
# RUN sudo apt-get install ros-humble-ros2bag ros-humble-rosbag2-transport
# RUN apt-get install ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap

# RUN sudo apt update & sudo apt install ros-rolling-rosbag2-storage-mcap
# TODO: fix for production
# RUN pip install utils/citros_cli
# RUN pip install LRS-Lulav
# END CITROS

RUN pip install citros
RUN pip install citros --upgrade




RUN chmod +x ros2_entrypoint.sh
ENTRYPOINT ["/app/ros2_entrypoint.sh"]

CMD ["bash"]
