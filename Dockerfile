FROM ros:foxy

# install ros package
RUN apt-get update && \
    apt-get install -y \
        ros-${ROS_DISTRO}-demo-nodes-cpp \
        ros-${ROS_DISTRO}-demo-nodes-py \
        python3-pip && \
    rm -rf /var/lib/apt/lists/*

COPY .env .env

ENV $(cat .env | grep -v ^# | xargs)

COPY requirements.txt .

RUN pip install --no-cache-dir -r ./requirements.txt
RUN apt-get update && apt-get install -y \
    python3-pyaudio \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /QA_ws

COPY src /QA_ws/src

RUN /bin/bash -c '. /opt/ros/foxy/setup.bash' 

# RUN colcon build --symlink-install && /bin/bash -c '. install/setup.bash'

# CMD ['']
    