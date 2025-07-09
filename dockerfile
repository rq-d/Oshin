FROM ardupilot/ardupilot-dev-ros:latest

# To make installation easy, we will clone the required repositories using vcs and a ros2.repos files:
RUN mkdir -p /root/ardu_ws/src \
  && cd /root/ardu_ws \
  && vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src

# Create and build MAVROS workspace
RUN cd /root/ardu_ws/src && git clone -b ros2 https://github.com/mavlink/mavros.git

# Now update all dependencies:
RUN cd /root/ardu_ws \
  && apt update  \
  && rosdep update \ 
  && cat /opt/ros/humble/setup.bash \
  && bash -c "cd /root/ardu_ws && source /opt/ros/humble/setup.bash" \
  && rosdep install --from-paths src --ignore-src -r -y 

# Install build tools and dependencies
RUN apt-get update && apt-get install -y \
  python3-colcon-common-extensions \
  ros-humble-ament-cmake \
  ros-humble-mavros-msgs \
  && rm -rf /var/lib/apt/lists/*

# Installing the MicroXRCEDDSGen build dependency:
RUN sudo apt install default-jre \
  && cd /root/ardu_ws \
  && git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git \
  && cd Micro-XRCE-DDS-Gen \
  && ./gradlew assemble \
  && echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc 

# Build the ros packages
RUN bash -c "source /opt/ros/humble/setup.bash && cd /root/ardu_ws && colcon build --symlink-install --parallel-workers 1"

# Mavros needs this installed
RUN apt-get update && apt-get install -y wget && \
  wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
  bash install_geographiclib_datasets.sh

RUN pip install -U MAVProxy

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# RUN echo "export ROS_DOMAIN_ID=99" >> ~/.bashrc

RUN apt-get update && apt-get install -y vim

WORKDIR /root
CMD ["/bin/bash"]