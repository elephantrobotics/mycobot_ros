ARG BASE_IMAGE
ARG ROS_DISTRO
FROM osrf/ros:${ROS_DISTRO}-desktop-full AS ros_distro

FROM ${BASE_IMAGE}
ARG ROS_DISTRO

# For this build, we pull the entire ros image, and then merge the filesystem
# with the nvidia/opengl image, so that displaying to the screen on GPU
# (or without GPU) works through docker.
COPY --from=ros_distro / /

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash", "-c"]

# Install build dependencies
RUN apt-get update && apt-get install -y curl
RUN PY_PKG_V=""; if [ "${ROS_DISTRO}" == "noetic" ]; then PY_PKG_V="3"; fi && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get install -y \
        sudo \
        # ROS Build dependencies
        python${PY_PKG_V}-rosinstall \
        python${PY_PKG_V}-rosinstall-generator \
        python${PY_PKG_V}-wstool \
        build-essential \
        python3-pip

RUN useradd -m er
RUN echo er:'Elephant' | chpasswd
RUN echo "er ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers
USER er
WORKDIR /home/er

ENV BASH_ENV="/home/er/launch.sh"
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $BASH_ENV

# Install python dependencies
ARG PYMYCOBOT_VERSION
RUN pip3 install "pymycobot $PYMYCOBOT_VERSION" --user

# Copy myCobot ROS package
WORKDIR /home/er/catkin_ws/src
COPY . mycobot_ros

USER root
WORKDIR /home/er/catkin_ws/src
RUN rosdep install -r -y -i --from-paths .

USER er
# Build the project
WORKDIR /home/er/catkin_ws
RUN catkin_make

# Let ROS know about the projects launch options
RUN echo "source /home/er/catkin_ws/devel/setup.bash" >> $BASH_ENV

# Setup the environment for interactive mode
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/er/.bashrc
RUN echo "source /home/er/catkin_ws/devel/setup.bash" >> /home/er/.bashrc
