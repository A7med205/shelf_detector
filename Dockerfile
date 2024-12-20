# Base image
FROM osrf/ros:humble-desktop-focal

# Install Gazebo 11 and other dependencies
RUN apt-get update && apt-get install -y \
  gazebo11 \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-gazebo-plugins \
  ros-humble-teleop-twist-keyboard \ 
  ros-humble-teleop-twist-joy \
  ros-humble-xacro \
  ros-humble-nav2* \
  ros-humble-urdf \
  ros-humble-urdf \
  python \
  && pip install boto3 assemblyai \
  && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p home/user/ros2_ws/src
RUN mkdir -p home/user/webpage_ws
WORKDIR home/user/ros2_ws

# Copy the files in the current directory into the container
COPY cartographer_slam home/user/ros2_ws/src/cartographer_slam
COPY localization_server home/user/ros2_ws/src/localization_server
COPY map_server home/user/ros2_ws/src/map_server
COPY path_planner_server home/user/ros2_ws/src/path_planner_server
COPY shelf_detector home/user/ros2_ws/src/shelf_detector
COPY /webpage_ws home/user/webpage_ws

# Building the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd home/user/ros2_ws && colcon build && source home/user/ros2_ws/install/setup.bash"

# Source the workspace at startup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source home/user/ros2_ws/install/setup.bash" >> ~/.bashrc

# Starting a bash shell
CMD ["bash"]