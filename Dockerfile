FROM ros:foxy

# install ros package
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y python3-pip

COPY .env .env

ENV $(cat .env | grep -v ^# | xargs)

COPY requirements.txt .

RUN pip install --no-cache-dir -r ./requirements.txt

WORKDIR /QA_ws

COPY src /QA_ws/src

# RUN /bin/bash -c '. /opt/ros/foxy/setup.bash' && \
#     colcon build --symlink-install 
    