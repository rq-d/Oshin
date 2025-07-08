FROM ghcr.io/sloretz/ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install required apt dependencies
RUN apt-get update && apt-get install -y \
  git python3-pip python3-colcon-common-extensions \
  libeigen3-dev libgeographic-dev geographiclib-tools \
  python3-vcstool python3-click libasio-dev \
  build-essential wget nano \
  ros-humble-mavlink \
  ros-humble-diagnostic-updater \
  ros-humble-eigen-stl-containers \
  ros-humble-geographic-msgs && \
  rm -rf /var/lib/apt/lists/*

# Install Python tools for MAVLink and MAVProxy
RUN pip3 install pymavlink MAVProxy

# Set up GeographicLib datasets
RUN geographiclib-get-geoids egm96-5

# Create and build MAVROS workspace
WORKDIR /root/ros2_ws/src
RUN git clone -b ros2 https://github.com/mavlink/mavros.git

# Clone angles manually into the workspace
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/ros/angles.git -b ros2

WORKDIR /root/ros2_ws
RUN rosdep update && \
  rosdep install --from-paths src --ignore-src -r -y

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install --parallel-workers 1"


RUN python3 -m pip install --no-cache-dir pexpect

# Clone and build ArduPilot SITL
WORKDIR /root
RUN git clone https://github.com/ArduPilot/ardupilot.git && \
  cd ardupilot && \
  git submodule update --init --recursive && \
  ./waf configure --board sitl && \
  ./waf build

# Add sourcing to bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
  echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /root
CMD ["/bin/bash"]