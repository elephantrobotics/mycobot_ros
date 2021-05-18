ARG BASE_IMAGE

FROM ${BASE_IMAGE}

# For this build, we pull the entire ros image, and then merge the filesystem
# with the nvidia/opengl image, so that displaying to the screen on GPU
# (or without GPU) works through docker.
# I was unable to use the ROS_DISTRO variable here due to this issue:
# https://github.com/docker/for-mac/issues/2155
COPY --from=osrf/ros:melodic-desktop-full / /

# Add ROS env vars to the bashrc
ENV BASH_ENV="/root/launch.sh"
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash", "-c"]
ARG ROS_DISTRO
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $BASH_ENV

# Install build dependencies
RUN apt-get update && \
    apt-get install -y \
        # ROS Build dependencies
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        build-essential \
        # Project-specific build dependencies
        python-pip \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-serial \
        ros-${ROS_DISTRO}-joint-state-publisher-gui && \
    rm -rf /var/lib/apt/lists/*

# Install python dependencies
ARG PYMYCOBOT_VERSION
RUN pip install "pymycobot $PYMYCOBOT_VERSION" --user

# Build the project
WORKDIR /catkin_ws/src
ADD . mycobot_ros
WORKDIR /catkin_ws
RUN catkin_make

# Let ROS know about the projects launch options
RUN echo "source /catkin_ws/devel/setup.bash" >> $BASH_ENV
