FROM ros:foxy

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /QA_ws

COPY src /QA_ws/src

RUN /bin/bash -c '. /opt/ros/foxy/setup.bash' && \
    colcon build --symlink-install 
    