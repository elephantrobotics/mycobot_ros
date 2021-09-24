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

# Copy myCobot ROS package
WORKDIR /catkin_ws/src
COPY . mycobot_ros

# Install build dependencies
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y \
        # ROS Build dependencies
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        build-essential \
        python3-pip && \
    # Project-specific build dependencies
    rosdep install -r -y -i --from-paths . && \
    rm -rf /var/lib/apt/lists/*

# Install python dependencies
ARG PYMYCOBOT_VERSION
RUN pip3 install "pymycobot $PYMYCOBOT_VERSION" --user

# Build the project
WORKDIR /catkin_ws
RUN catkin_make

# Let ROS know about the projects launch options
RUN echo "source /catkin_ws/devel/setup.bash" >> $BASH_ENV
