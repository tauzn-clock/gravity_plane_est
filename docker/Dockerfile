FROM nvidia/cuda:11.4.3-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    bash \
    lsb-release \
    git \
    curl \
    python3 \
    python3-dev \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    ros-noetic-grid-map \
    ros-noetic-grid-map-msgs \
    ros-noetic-pybind11-catkin \
    libgmp-dev \
    libmpfr-dev \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-catkin-tools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src
RUN git clone https://github.com/tauzn-clock/gravity_plane_est.git
RUN git clone https://github.com/leggedrobotics/elevation_mapping_cupy.git

# Set the shell to bash
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"
ENTRYPOINT ["/bin/bash"]
